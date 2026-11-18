"""작업 상태머신 ROS 노드.

로직은 machine.TaskMachine에 있고, 이 노드는 ROS 연결만 한다:
액션 · 서비스 호출, TF 조회, 타이머, /cmd_vel 선택 발행 (docs/interfaces.md 5.0).
nav_only false면 도착 확인 뒤 /servoing/align(D) → /grasp/execute(B)까지 진행한다 (4장).
Grasp 목표의 target_pose는 /perception/target_pose(C) 중 정렬 뒤 찍힌 1 s 이내 값 (target_pose.py).
정렬 중에는 같은 토픽의 물체 수평거리로 Near→Pre-grasp 전환을 판단한다 (pose_transition.py).
전환 요청 방식은 D·B 확인 전이라 요청은 로그만 남긴다.
"""

import datetime
import math
import time

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal as CancelGoalSrv
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from project_interfaces.action import Align, Grasp
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMsg
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from mm_task_manager import codes
from mm_task_manager import machine as sm
from mm_task_manager.cmd_select import NAV, SERVO, CmdSelector, source_for
from mm_task_manager.geometry import yaw_from_quaternion
from mm_task_manager.mission import Mission, mission_from_dict
from mm_task_manager.align_distance import target_distance_event
from mm_task_manager.result_log import append_jsonl
from mm_task_manager.final_stop import FinalStop
from mm_task_manager.run import NodeExit, run
from mm_task_manager.start_gate import can_start
from mm_task_manager.state_qos import state_qos
from mm_task_manager.states import TERMINAL_STATES, State, StateReporter
from mm_task_manager.target_pose import SEND, STALE, GraspPoseRequest, is_valid_target_pose

LIFECYCLE_SERVICES = ('/lifecycle_manager_localization/is_active',
                      '/lifecycle_manager_navigation/is_active')

_NAV_STATUS = {
    GoalStatus.STATUS_SUCCEEDED: sm.NAV_SUCCEEDED,
    GoalStatus.STATUS_ABORTED: sm.NAV_ABORTED,
    GoalStatus.STATUS_CANCELED: sm.NAV_CANCELED,
}


def _stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


# 이동 목표 파라미터: goal.*(직접 지정)와 object.*(물체 위치) 중 YAML에 적은 쪽만 넘긴다
_NAV_TARGET_PARAMS = {'goal.x': 'goal_x', 'goal.y': 'goal_y', 'goal.yaw': 'goal_yaw',
                      'object.x': 'object_x', 'object.y': 'object_y'}


def _declare_mission_params(node):
    """Mission 필드를 ROS 파라미터로 선언하고 dict로 읽는다.

    goal.* / object.*는 NaN을 기본값으로 선언하고, NaN(=YAML에 없음)인 항목은 넘기지 않는다.
    둘 중 정확히 하나인지는 mission_from_dict가 확인한다.
    """
    defaults = {f: getattr(Mission('x'), f) for f in Mission.__dataclass_fields__
                if f not in _NAV_TARGET_PARAMS.values() and f != 'target_class'}
    params = {'target_class': node.declare_parameter('target_class', '').value}
    for name in _NAV_TARGET_PARAMS:
        value = node.declare_parameter(name, math.nan).value
        if not math.isnan(value):
            params[name] = value
    for name, default in defaults.items():
        params[name] = node.declare_parameter(name, default).value
    return params


class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('task_manager')
        mission, warnings = mission_from_dict(_declare_mission_params(self))
        for w in warnings:
            self.get_logger().warn(w)
        self.mission = mission
        self.autostart = self.declare_parameter('autostart', True).value
        # /task_manager/state 구독자가 이 수만큼 연결된 뒤 출발 (mock 시험용, 기본 0 = 기다리지 않음)
        self.wait_state_subscribers = self.declare_parameter('wait_state_subscribers', 0).value

        run_id = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        self.machine = sm.TaskMachine(mission, run_id)
        self.selector = CmdSelector(mission.cmd_input_timeout_sec)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 토픽
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.class_pub = self.create_publisher(String, '/perception/target_class', latched)
        # 현재 단계 이름 (interfaces.md v0.14 3장): 바뀔 때만 발행, transient_local depth 10 (state_qos.py)
        self.state_pub = self.create_publisher(String, '/task_manager/state', state_qos())
        self.state_reporter = StateReporter()
        self._report_state()
        self.create_subscription(Twist, '/nav2/cmd_vel',
                                 lambda msg: self.selector.update(NAV, msg, self._now()), 10)
        self.create_subscription(Twist, '/servoing/cmd_vel',
                                 lambda msg: self.selector.update(SERVO, msg, self._now()), 10)

        # 시작 확인
        self.lifecycle_clients = {s: self.create_client(Trigger, s) for s in LIFECYCLE_SERVICES}
        self.lifecycle_active = {s: None for s in LIFECYCLE_SERVICES}
        self.lifecycle_pending = set()
        self.last_lifecycle_poll = -math.inf

        # 액션 (interfaces.md 4장). goal_handles는 세 액션이 token으로 같이 씀
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.align_client = ActionClient(self, Align, '/servoing/align')
        self.grasp_client = ActionClient(self, Grasp, '/grasp/execute')
        self.goal_handles = {}
        self.cancel_requested = set()

        # Grasp에 넘길 물체 위치 (C, base_link). 정렬 뒤 찍힌 1 s 이내 값이 올 때까지 Grasp 전송 대기
        self.latest_target_pose = None
        self.grasp_request = None
        self.create_subscription(PoseStamped, '/perception/target_pose',
                                 self._on_target_pose, 10)

        # /clock 감시 (wall time: sim time 타이머는 /clock이 멈추면 같이 멈춤)
        self.last_clock_wall = None
        self.last_clock_sim = 0.0
        self.clock_stop_reported = False
        self.create_subscription(ClockMsg, '/clock', self._on_clock, 10)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.5, self._check_clock, clock=self.steady)

        # 종료 처리: 끝나면 0을 final_zero_sec(wall) 동안 보낸 뒤 노드 종료
        self.final_stop = FinalStop(mission.final_zero_sec)

        # sim time 타이머
        self.create_timer(0.1, self._tick)
        self.create_timer(1.0 / mission.cmd_rate_hz, self._publish_cmd)

        if mission.nav_goal_mode == 'object':
            target = (f'object ({mission.object_x}, {mission.object_y}), '
                      f'approach distance {mission.approach_distance:.3f}')
        else:
            target = f'goal ({mission.goal_x}, {mission.goal_y}, {mission.goal_yaw})'
        self.get_logger().info(
            f'run {run_id}: {target}, '
            f'target_class {mission.target_class}, nav_only {mission.nav_only}')

    # ---- 시각 ----

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_clock(self, msg):
        self.last_clock_wall = time.monotonic()
        self.last_clock_sim = _stamp_sec(msg.clock)

    def _check_clock(self):
        if self.last_clock_wall is None or self.clock_stop_reported:
            return
        silent = time.monotonic() - self.last_clock_wall
        if silent > self.mission.clock_watchdog_sec:
            self.clock_stop_reported = True
            self.get_logger().error(f'/clock stopped for {silent:.1f} s (wall)')
            self.cmd_pub.publish(Twist())           # sim time 타이머가 멈췄으므로 직접 0 발행
            self._dispatch(sm.ClockStopped(now=self.last_clock_sim))

    # ---- 주기 처리 ----

    def _tick(self):
        now = self._now()
        state = self.machine.state
        if state == State.IDLE:
            if can_start(self.autostart, now, self.state_pub.get_subscription_count(),
                         self.wait_state_subscribers):
                self._dispatch(sm.Start(now=now))
            return
        if state == State.WAIT_READY:
            self._poll_lifecycle(now)
            self._dispatch(sm.ReadyStatus(
                now=now,
                localization_active=self.lifecycle_active[LIFECYCLE_SERVICES[0]],
                navigation_active=self.lifecycle_active[LIFECYCLE_SERVICES[1]],
                nav_server_ready=self.nav_client.server_is_ready(),
                map_odom_stamp=self._map_odom_stamp(),
                cmd_vel_other_publishers=max(0, self.count_publishers('/cmd_vel') - 1),
                robot_pose=self._map_base_pose()))
        elif state == State.VERIFY_ARRIVAL:
            self._dispatch(sm.PoseSample(now=now, pose=self._map_base_pose(),
                                         map_odom_stamp=self._map_odom_stamp()))
        elif state == State.GRASP:
            self._check_grasp_request(now)
        self._dispatch(sm.Tick(now=now))

    def _publish_cmd(self):
        cmd = self.selector.output(self._now())
        self.cmd_pub.publish(cmd if cmd is not None else Twist())

    def _poll_lifecycle(self, now):
        if now - self.last_lifecycle_poll < 1.0:
            return
        self.last_lifecycle_poll = now
        for name, client in self.lifecycle_clients.items():
            if name in self.lifecycle_pending or not client.service_is_ready():
                continue
            self.lifecycle_pending.add(name)
            future = client.call_async(Trigger.Request())
            future.add_done_callback(lambda f, n=name: self._on_lifecycle(n, f))

    def _on_lifecycle(self, name, future):
        self.lifecycle_pending.discard(name)
        result = future.result()
        self.lifecycle_active[name] = bool(result.success) if result is not None else None

    # ---- TF ----

    def _map_odom_stamp(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'odom', Time())
        except TransformException:
            return None
        return _stamp_sec(t.header.stamp)

    def _map_base_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return (tr.x, tr.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    # ---- 상태머신 이벤트 · 명령 ----

    def _dispatch(self, event):
        prev = self.machine.state
        for cmd in self.machine.handle(event):
            self._execute(cmd)
        state = self.machine.state
        self.selector.select(source_for(state), self._now())
        if state != prev:
            self.get_logger().info(f'{prev.value} -> {state.value}')
        self._report_state()
        if state in TERMINAL_STATES and not self.final_stop.started:
            self.final_stop.start(time.monotonic())
            self.get_logger().info(f'sending zero for {self.mission.final_zero_sec} s, then exit')
            self.create_timer(0.1, self._final_stop_tick, clock=self.steady)

    def _report_state(self):
        name = self.state_reporter.update(self.machine.state)
        if name is not None:
            self.state_pub.publish(String(data=name))

    def _final_stop_tick(self):
        # sim time 타이머와 별개로 wall time으로 0 발행 (/clock이 멈춰도 동작)
        self.cmd_pub.publish(Twist())
        if self.final_stop.should_exit(time.monotonic()):
            self.get_logger().info('final stop done, exiting')
            raise NodeExit()

    def _execute(self, cmd):
        if isinstance(cmd, sm.PublishTargetClass):
            self.class_pub.publish(String(data=cmd.target_class))
        elif isinstance(cmd, sm.SendNavGoal):
            self._send_nav_goal(cmd)
        elif isinstance(cmd, sm.SendAlignGoal):
            self._send_align_goal(cmd)
        elif isinstance(cmd, sm.SendGraspGoal):
            # 바로 보내지 않고, 정렬 뒤 찍힌 target_pose가 오면 보냄 (_check_grasp_request)
            self.grasp_request = GraspPoseRequest(cmd.token, self._now(), cmd.max_pose_age_sec)
            self._check_grasp_request(self._now())
        elif isinstance(cmd, sm.RequestObservePose):
            # TODO: PR #1 답변 대기 중 (D·B). 관측 자세 요청 액션이 확정되면 여기서 실제로 보낸다
            self.get_logger().warn(
                f'observe pose {cmd.pose_name} requested at {cmd.distance_m:.3f} m '
                f'(< {self.mission.pregrasp_switch_distance_m:.2f} m), not sent: '
                'request interface not decided yet')
        elif isinstance(cmd, sm.CancelGoal):
            self._cancel(cmd.token)
        elif isinstance(cmd, sm.WriteRecord):
            self._write_record(cmd.record)
        else:
            self.get_logger().error(f'unsupported command {cmd}')

    def _send_nav_goal(self, cmd):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = cmd.x
        goal.pose.pose.position.y = cmd.y
        goal.pose.pose.orientation.z = math.sin(cmd.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(cmd.yaw / 2.0)
        self.get_logger().info(
            f'nav goal #{cmd.token}: ({cmd.x:.3f}, {cmd.y:.3f}, {cmd.yaw:.3f})')
        self._send_goal(self.nav_client, goal, cmd.token, self._on_nav_result,
                        lambda t: sm.NavResult(now=self._now(), token=t, status=sm.NAV_REJECTED))

    def _send_align_goal(self, cmd):
        goal = Align.Goal(target_class=cmd.target_class, standoff_m=cmd.standoff_m,
                          timeout_sec=cmd.timeout_sec)
        self.get_logger().info(f'align goal #{cmd.token}: {cmd.target_class} '
                               f'standoff {cmd.standoff_m:.3f} timeout {cmd.timeout_sec:.1f}')
        self._send_goal(self.align_client, goal, cmd.token,
                        lambda t, f: self._on_task_result(sm.AlignResult, 'align', t, f),
                        lambda t: _rejected(sm.AlignResult, self._now(), t))

    def _on_target_pose(self, msg):
        p = msg.pose.position
        frame_id = msg.header.frame_id
        if not is_valid_target_pose(frame_id, p.x, p.y, p.z):
            # 안전장치(C 제안): Grasp 목표 후보에도, 전환 거리 판단에도 안 씀
            self.get_logger().warn(
                f'ignored target_pose: frame {frame_id!r} ({p.x}, {p.y}, {p.z})',
                throttle_duration_sec=5.0)
            return
        self.latest_target_pose = msg
        if self.machine.state == State.GRASP:
            self._check_grasp_request(self._now())
        ev = target_distance_event(self.machine.state, frame_id, p.x, p.y, p.z, self._now())
        if ev is not None:
            self._dispatch(ev)

    def _check_grasp_request(self, now):
        req = self.grasp_request
        if req is None:
            return
        pose = self.latest_target_pose
        decision = req.check(_stamp_sec(pose.header.stamp) if pose is not None else None, now)
        if decision == SEND:
            self.grasp_request = None
            self._send_grasp_goal(req.token, pose)
        elif decision == STALE:
            self.grasp_request = None
            self.get_logger().warn(f'no target_pose within {req.max_age_sec:.1f} s after align')
            self._dispatch(sm.GraspResult(now=now, token=req.token, success=False,
                                          error_code=codes.OBJECT_NOT_FOUND,
                                          detail=codes.TARGET_POSE_STALE))

    def _send_grasp_goal(self, token, pose):
        goal = Grasp.Goal(target_class=self.mission.target_class, target_pose=pose)
        p = pose.pose.position
        self.get_logger().info(f'grasp goal #{token}: {goal.target_class} '
                               f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) {pose.header.frame_id}')
        self._send_goal(self.grasp_client, goal, token,
                        lambda t, f: self._on_task_result(sm.GraspResult, 'grasp', t, f),
                        lambda t: _rejected(sm.GraspResult, self._now(), t))

    def _send_goal(self, client, goal, token, on_result, on_rejected):
        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_accepted(token, f, on_result, on_rejected))

    def _on_accepted(self, token, future, on_result, on_rejected):
        handle = future.result()
        if handle is None or not handle.accepted:
            self._dispatch(on_rejected(token))
            return
        self.goal_handles[token] = handle
        handle.get_result_async().add_done_callback(lambda f, t=token: on_result(t, f))
        if token in self.cancel_requested:     # 수락 전에 취소 요청이 있었음
            self._cancel(token)

    def _on_nav_result(self, token, future):
        status = future.result().status
        self.get_logger().info(f'nav goal #{token} result status {status}')
        self._dispatch(sm.NavResult(now=self._now(), token=token,
                                    status=_NAV_STATUS.get(status, sm.NAV_ABORTED)))

    def _on_task_result(self, result_type, name, token, future):
        # Align/Grasp: 성공 여부와 코드는 Result 필드로 판단. 취소된 경우 detail canceled
        res = future.result()
        detail = codes.CANCELED if res.status == GoalStatus.STATUS_CANCELED else ''
        self.get_logger().info(f'{name} goal #{token} status {res.status} '
                               f'success {res.result.success} code {res.result.error_code!r}')
        self._dispatch(result_type(now=self._now(), token=token, success=bool(res.result.success),
                                   error_code=res.result.error_code, detail=detail))

    def _cancel(self, token):
        self.cancel_requested.add(token)
        if self.grasp_request is not None and self.grasp_request.token == token:
            self.grasp_request = None           # 아직 보내지 않은 Grasp: 보낼 것이 없음
            self._dispatch(sm.CancelDone(now=self._now(), token=token))
            return
        handle = self.goal_handles.get(token)
        if handle is None:
            return                              # 수락 응답이 오면 그때 취소
        self.get_logger().warn(f'canceling goal #{token}')
        handle.cancel_goal_async().add_done_callback(
            lambda f, t=token: self._on_cancel_response(t, f))

    def _on_cancel_response(self, token, future):
        resp = future.result()
        if resp.return_code == CancelGoalSrv.Response.ERROR_NONE and resp.goals_canceling:
            self._dispatch(sm.CancelDone(now=self._now(), token=token))
        else:
            # 거부된 취소는 완료로 보지 않음: 목표가 계속 돌 수 있으므로 cancel_timeout까지 기다림
            self.get_logger().warn(f'cancel of goal #{token} not accepted '
                                   f'(return_code {resp.return_code})')

    def _write_record(self, record):
        record = dict(record, wall_time=datetime.datetime.now().isoformat(timespec='seconds'))
        try:
            append_jsonl(self.mission.result_log_path, record)
        except OSError as e:
            self.get_logger().error(f'result log write failed: {e}')
        self.get_logger().info(
            f'finished {record["final_state"]} error_code={record["error_code"]!r} '
            f'detail={record["detail"]!r} stage={record["failed_stage"]!r}')


def _rejected(result_type, now, token):
    return result_type(now=now, token=token, success=False, error_code=codes.UNKNOWN,
                       detail=codes.REJECTED)


def main(args=None):
    run(TaskManagerNode, args, on_exit=lambda node: node.cmd_pub.publish(Twist()))
