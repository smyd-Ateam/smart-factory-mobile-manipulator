"""작업 상태머신 ROS 노드.

로직은 machine.TaskMachine에 있고, 이 노드는 ROS 연결만 한다:
액션 · 서비스 호출, TF 조회, 타이머, /cmd_vel 선택 발행 (docs/interfaces.md 5.0).
"""

import datetime
import math
import time

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal as CancelGoalSrv
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMsg
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from mm_task_manager import machine as sm
from mm_task_manager.cmd_select import NAV, SERVO, CmdSelector, source_for
from mm_task_manager.geometry import yaw_from_quaternion
from mm_task_manager.mission import Mission, mission_from_dict
from mm_task_manager.result_log import append_jsonl
from mm_task_manager.run import run
from mm_task_manager.states import State

LIFECYCLE_SERVICES = ('/lifecycle_manager_localization/is_active',
                      '/lifecycle_manager_navigation/is_active')

_NAV_STATUS = {
    GoalStatus.STATUS_SUCCEEDED: sm.NAV_SUCCEEDED,
    GoalStatus.STATUS_ABORTED: sm.NAV_ABORTED,
    GoalStatus.STATUS_CANCELED: sm.NAV_CANCELED,
}


def _stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _declare_mission_params(node):
    """Mission 필드를 ROS 파라미터로 선언하고 dict로 읽는다."""
    defaults = {f: getattr(Mission(0.0, 0.0, 0.0, 'x'), f)
                for f in Mission.__dataclass_fields__}
    for f in ('goal_x', 'goal_y', 'goal_yaw', 'target_class'):
        defaults.pop(f)
    params = {}
    for name, default in [('goal.x', math.nan), ('goal.y', math.nan), ('goal.yaw', math.nan),
                          ('target_class', '')] + list(defaults.items()):
        params[name] = node.declare_parameter(name, default).value
    return params


class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('task_manager')
        mission, warnings = mission_from_dict(_declare_mission_params(self))
        for w in warnings:
            self.get_logger().warn(w)
        if not mission.nav_only:
            # TODO(project_interfaces): Align/Grasp 액션 클라이언트 (align_grasp_clients.py)
            raise RuntimeError('nav_only:=false needs project_interfaces (not created yet)')
        self.mission = mission
        self.autostart = self.declare_parameter('autostart', True).value

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
        self.create_subscription(Twist, '/nav2/cmd_vel',
                                 lambda msg: self.selector.update(NAV, msg, self._now()), 10)
        self.create_subscription(Twist, '/servoing/cmd_vel',
                                 lambda msg: self.selector.update(SERVO, msg, self._now()), 10)

        # 시작 확인
        self.lifecycle_clients = {s: self.create_client(Trigger, s) for s in LIFECYCLE_SERVICES}
        self.lifecycle_active = {s: None for s in LIFECYCLE_SERVICES}
        self.lifecycle_pending = set()
        self.last_lifecycle_poll = -math.inf

        # Nav2 액션
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handles = {}
        self.cancel_requested = set()

        # /clock 감시 (wall time: sim time 타이머는 /clock이 멈추면 같이 멈춤)
        self.last_clock_wall = None
        self.last_clock_sim = 0.0
        self.clock_stop_reported = False
        self.create_subscription(ClockMsg, '/clock', self._on_clock, 10)
        self.create_timer(0.5, self._check_clock,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

        # sim time 타이머
        self.create_timer(0.1, self._tick)
        self.create_timer(1.0 / mission.cmd_rate_hz, self._publish_cmd)

        self.get_logger().info(
            f'run {run_id}: goal ({mission.goal_x}, {mission.goal_y}, {mission.goal_yaw}), '
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
            if self.autostart and now > 0.0:
                self._dispatch(sm.Start(now=now))
            return
        if state == State.WAIT_READY:
            self._poll_lifecycle(now)
            self._dispatch(sm.ReadyStatus(
                now=now,
                localization_active=self.lifecycle_active[LIFECYCLE_SERVICES[0]],
                navigation_active=self.lifecycle_active[LIFECYCLE_SERVICES[1]],
                nav_server_ready=self.nav_client.server_is_ready(),
                map_odom_stamp=self._map_odom_stamp()))
        elif state == State.VERIFY_ARRIVAL:
            self._dispatch(sm.PoseSample(now=now, pose=self._map_base_pose(),
                                         map_odom_stamp=self._map_odom_stamp()))
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

    def _execute(self, cmd):
        if isinstance(cmd, sm.PublishTargetClass):
            self.class_pub.publish(String(data=cmd.target_class))
        elif isinstance(cmd, sm.SendNavGoal):
            self._send_nav_goal(cmd)
        elif isinstance(cmd, sm.CancelGoal):
            self._cancel(cmd.token)
        elif isinstance(cmd, sm.WriteRecord):
            self._write_record(cmd.record)
        else:
            # SendAlignGoal, SendGraspGoal: nav_only에서는 나오지 않음
            self.get_logger().error(f'unsupported command {cmd}')

    def _send_nav_goal(self, cmd):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = cmd.x
        goal.pose.pose.position.y = cmd.y
        goal.pose.pose.orientation.z = math.sin(cmd.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(cmd.yaw / 2.0)
        self.get_logger().info(f'nav goal #{cmd.token}: ({cmd.x}, {cmd.y}, {cmd.yaw})')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f, t=cmd.token: self._on_nav_accepted(t, f))

    def _on_nav_accepted(self, token, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self._dispatch(sm.NavResult(now=self._now(), token=token, status=sm.NAV_REJECTED))
            return
        self.goal_handles[token] = handle
        handle.get_result_async().add_done_callback(
            lambda f, t=token: self._on_nav_result(t, f))
        if token in self.cancel_requested:     # 수락 전에 취소 요청이 있었음
            self._cancel(token)

    def _on_nav_result(self, token, future):
        status = future.result().status
        self.get_logger().info(f'nav goal #{token} result status {status}')
        self._dispatch(sm.NavResult(now=self._now(), token=token,
                                    status=_NAV_STATUS.get(status, sm.NAV_ABORTED)))

    def _cancel(self, token):
        self.cancel_requested.add(token)
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


def main(args=None):
    run(TaskManagerNode, args, on_exit=lambda node: node.cmd_pub.publish(Twist()))
