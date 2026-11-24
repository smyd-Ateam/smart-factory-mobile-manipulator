"""베이스 미세 정렬 액션 서버 /servoing/align (project_interfaces/action/Align).

입력: /perception/detections, /camera/depth/image_raw, /camera/camera_info, TF
출력: /servoing/cmd_vel (목표 실행 중에만, 끝나면 0 한 번)

목표 진행 판단(완료 판정, 시간 초과, 물체 놓침)은 session.AlignSession이 한다.
"""
import signal
import threading
import time
from collections import deque

import rclpy
from rclpy.exceptions import ROSInterruptException
from geometry_msgs.msg import Twist
from project_interfaces.action import Align
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray

from visual_servoing.control import ControlParams
from visual_servoing.geometry import decode_depth, transform_point
from visual_servoing.object_model import Cylinder, parse_object_shapes
from visual_servoing.session import RUNNING, SUCCEEDED, AlignSession, SessionParams
from visual_servoing.tracking import (
    Candidate, base_bearing, candidate_base_xy, pick_near_expected, pick_target)


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


# 클래스:반지름:높이 (m), 세워진 원기둥으로 근사. feat/yolo 부품 치수 기준
# block 2.5×2.5×4 cm, pin 지름 2 × 5 cm, hex_nut 맞변 3 × 1.5 cm, bolt 몸통 지름 1.2 × 전체 6 cm
DEFAULT_OBJECT_SHAPES = ['block:0.0125:0.04', 'pin:0.010:0.05',
                         'hex_nut:0.015:0.015', 'bolt:0.006:0.06']
NO_SHAPE = Cylinder(0.0, 1.0)   # 모르는 클래스: 잰 점을 그대로 씀
# 예상 위치를 base_link로 옮기지 못했을 때 결과 코드. interfaces.md 4.1 "별도 TF 오류".
# 5.3 실패 코드 표에 이름이 아직 없어 임시 이름 (A 확인 후 확정)
TF_ERROR = 'TF_ERROR'
# 깊이·camera_info는 RELIABLE로 받는다. 약 1 MB 이미지를 BEST_EFFORT(sensor_data)로 받으면
# ros_gz_bridge(RELIABLE 발행)에서 오는 프레임이 WSL에서도 40~60 %만 도착했다 (RELIABLE은 100 %,
# PR #1 RGB-D 브리지 진단, 2026-11-22).
CAMERA_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                        reliability=ReliabilityPolicy.RELIABLE)


class AlignNode(Node):

    def __init__(self):
        super().__init__('servoing_align')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('score_min', 0.5)
        self.declare_parameter('depth_half_window', 3)
        self.declare_parameter('depth_sync_tolerance', 0.02)   # s
        self.declare_parameter('control_rate', 20.0)           # Hz
        self.declare_parameter('feedback_rate', 5.0)           # Hz
        # 깊이로 잰 표면 점 → 물체 중심축 계산용 (object_model.py)
        self.declare_parameter('object_shapes', DEFAULT_OBJECT_SHAPES)
        self.declare_parameter('floor_z', -0.05)   # base_link 기준 바닥 높이 (mm_description)
        # 같은 클래스가 여러 개일 때 직전 박스에서 이만큼(px) 넘게 떨어진 검출로는 갈아타지 않음
        self.declare_parameter('track_gate_px', 150.0)
        # 예상 위치에 가장 가까운 두 후보의 거리 차이가 이보다 작으면 어느 쪽인지 정하지 않음
        # (m, interfaces.md 4.1 잠정값, 같은 클래스 2물체 실측 뒤 재검토)
        self.declare_parameter('expected_ambiguity_m', 0.03)
        # 정렬 시작 때 예상 위치 TF(map → base_link 등)를 기다리는 최대 시간 (s)
        self.declare_parameter('expected_tf_timeout', 1.0)
        for name, value in vars(ControlParams()).items():
            self.declare_parameter(name, value)
        for name, value in vars(SessionParams()).items():
            self.declare_parameter(name, value)

        self.object_shapes = parse_object_shapes(self.param('object_shapes'))
        self.depth_cache = deque(maxlen=15)   # (stamp_ns, frame_id, depth 배열)
        self.camera_info = None
        self.pending_detection = None         # 깊이보다 먼저 도착한 검출
        self.last_uv = None                   # 실행 중인 목표에서 마지막으로 고른 박스 중심 (px)
        self.lock = threading.Lock()
        self.active_target = None             # 실행 중인 목표의 target_class
        self.expected = None                  # 실행 중인 목표의 (PointStamped, 반경 m), 없으면 None
        self.measurement = None               # (stamp 초, x_b, y_b), base_link 기준
        self.shutting_down = threading.Event()   # 종료 중이면 실행 중인 목표를 멈춤
        self.idle = threading.Event()             # 실행 중인 목표가 없음
        self.idle.set()

        group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, '/servoing/cmd_vel', 10)
        self.create_subscription(Detection2DArray, '/perception/detections',
                                 self.on_detections, 10, callback_group=group)
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self.on_depth, CAMERA_QOS, callback_group=group)
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self.on_camera_info, CAMERA_QOS,
                                 callback_group=group)

        self.server = ActionServer(
            self, Align, '/servoing/align',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.get_logger().info('Align 액션 서버 준비: /servoing/align')

    def param(self, name):
        return self.get_parameter(name).value

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ---------- 입력 ----------

    def on_camera_info(self, msg):
        self.camera_info = msg

    def on_depth(self, msg):
        try:
            depth = decode_depth(msg.encoding, msg.data, msg.height, msg.width,
                                 msg.step, bool(msg.is_bigendian))
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        self.depth_cache.append((stamp_ns(msg.header.stamp), msg.header.frame_id, depth))
        # 토픽끼리 도착 순서가 보장되지 않으므로, 깊이보다 먼저 온 검출을 다시 처리한다.
        pending, self.pending_detection = self.pending_detection, None
        if pending is not None:
            self.on_detections(pending, retry=False)

    def find_depth(self, t_ns):
        tol = self.param('depth_sync_tolerance') * 1e9
        best = None
        for d_ns, frame, depth in list(self.depth_cache):
            diff = abs(d_ns - t_ns)
            if diff <= tol and (best is None or diff < best[0]):
                best = (diff, depth)
        return None if best is None else best[1]

    def half_window(self, det):
        """깊이 중앙값을 낼 창 반폭 (px). 작은 박스에서 배경 깊이가 섞이지 않게 박스 크기로 제한."""
        return max(1, min(self.param('depth_half_window'),
                          int(det.bbox.size_x / 4), int(det.bbox.size_y / 4)))

    def pick_detection(self, msg, target, cam_t, cam_q, depth):
        """목표 클래스 검출 중 따라가던 물체 하나를 고른다 (tracking.pick_target)."""
        score_min = self.param('score_min')
        dets = [d for d in msg.detections
                if d.results and d.results[0].hypothesis.class_id == target
                and d.results[0].hypothesis.score >= score_min]
        cands = [Candidate(d.bbox.center.position.x, d.bbox.center.position.y,
                           d.results[0].hypothesis.score) for d in dets]
        with self.lock:
            last_uv = self.last_uv
            expected = self.expected
        first_cost = []
        if last_uv is None:
            k = self.camera_info.k
            kk = (k[0], k[4], k[2], k[5])
            shape = self.object_shapes.get(target, NO_SHAPE)
            if expected is not None:   # 첫 프레임: 예상 위치 반경 안에서 가장 가까운 물체
                points = [candidate_base_xy(c.u, c.v, self.half_window(d), depth, kk, cam_t,
                                            cam_q, shape, self.param('floor_z'))
                          for c, d in zip(cands, dets)]
                i = self.pick_expected(points, expected)
                return self.remember_pick(target, cands, dets, i)
            # 첫 프레임: 차체 정면에 가장 가까운 물체
            z_mid = self.param('floor_z') + min(shape.height, 1.0) / 2.0
            first_cost = [abs(base_bearing(c.u, c.v, kk, cam_t, cam_q, z_mid)) for c in cands]
        i = pick_target(cands, last_uv, self.param('track_gate_px'), first_cost)
        return self.remember_pick(target, cands, dets, i)

    def pick_expected(self, points, expected):
        """base_link로 옮겨 둔 예상 위치 반경 안에서 후보를 고른다 (interfaces.md 4.1). 못 고르면 None."""
        (ex, ey), radius = expected
        i = pick_near_expected(points, (ex, ey), radius, self.param('expected_ambiguity_m'))
        if i is None and points:
            self.get_logger().warn(
                f'예상 위치 ({ex:.3f}, {ey:.3f}) 반경 {radius:.3f} m 안에서 목표를 정하지 못함 '
                f'(후보 {len(points)}개)', throttle_duration_sec=1.0)
        return i

    def remember_pick(self, target, cands, dets, i):
        """고른 박스 중심을 기억하고 그 검출을 돌려준다. i가 None이면 None."""
        if i is None:
            return None
        with self.lock:
            if self.active_target == target:
                self.last_uv = (cands[i].u, cands[i].v)
        return dets[i]

    def on_detections(self, msg, retry=True):
        target = self.active_target
        if target is None:
            return   # 실행 중인 목표가 없으면 계산하지 않음
        if self.camera_info is None:
            self.get_logger().warn('camera_info 대기 중', throttle_duration_sec=5.0)
            return
        t_ns = stamp_ns(msg.header.stamp)
        frame = msg.header.frame_id or self.camera_info.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(
                self.param('base_frame'), frame, Time(nanoseconds=t_ns),
                timeout=Duration(seconds=0.05))
        except TransformException as e:
            self.get_logger().warn(f'TF 실패: {e}', throttle_duration_sec=5.0)
            return
        tr, q = tf.transform.translation, tf.transform.rotation
        cam_t, cam_q = (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w)

        # 예상 위치로 후보를 고를 때도 깊이가 필요해서 고르기 전에 같은 stamp 깊이를 찾는다
        depth = self.find_depth(t_ns)
        if depth is None:
            if retry:
                self.pending_detection = msg   # 깊이가 오면 다시 처리
            else:
                self.get_logger().warn('같은 stamp의 깊이 이미지 없음', throttle_duration_sec=5.0)
            return

        det = self.pick_detection(msg, target, cam_t, cam_q, depth)
        if det is None:
            return

        k = self.camera_info.k
        # 깊이는 bbox 중심이 닿은 표면까지라 물체 모양으로 중심축을 구한다 (object_model.py)
        xy = candidate_base_xy(det.bbox.center.position.x, det.bbox.center.position.y,
                               self.half_window(det), depth, (k[0], k[4], k[2], k[5]),
                               cam_t, cam_q, self.object_shapes.get(target, NO_SHAPE),
                               self.param('floor_z'))
        if xy is None:
            self.get_logger().warn('bbox 중심에 유효한 깊이 없음', throttle_duration_sec=5.0)
            return
        x_b, y_b = xy
        with self.lock:
            if self.active_target == target:
                self.measurement = (t_ns / 1e9, x_b, y_b)

    # ---------- 액션 ----------

    def on_goal(self, goal):
        if self.shutting_down.is_set():
            return GoalResponse.REJECT
        if self.active_target is not None:
            self.get_logger().warn('이미 실행 중인 목표가 있어 새 목표 거부')
            return GoalResponse.REJECT
        if not goal.target_class or goal.standoff_m <= 0.0 or goal.timeout_sec <= 0.0:
            self.get_logger().warn(
                f'잘못된 목표 거부: target_class={goal.target_class!r}, '
                f'standoff_m={goal.standoff_m}, timeout_sec={goal.timeout_sec}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def expected_in_base(self, point):
        """예상 위치(PointStamped)를 최신 TF로 base_link (x, y)로 옮긴다. 실패하면 None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.param('base_frame'), point.header.frame_id, Time(),
                timeout=Duration(seconds=self.param('expected_tf_timeout')))
        except TransformException as e:
            self.get_logger().error(f'예상 위치 TF 실패 ({point.header.frame_id}): {e}')
            return None
        tr, q = tf.transform.translation, tf.transform.rotation
        x, y, _ = transform_point((point.point.x, point.point.y, point.point.z),
                                  (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
        return x, y

    def publish_cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def execute(self, goal_handle):
        goal = goal_handle.request
        self.idle.clear()
        with self.lock:
            self.active_target = goal.target_class
            self.measurement = None
            self.last_uv = None
            self.expected = None
        where = ''
        if goal.expected_position.header.frame_id:
            p = goal.expected_position
            where = (f', 예상 위치 ({p.point.x:.3f}, {p.point.y:.3f}) {p.header.frame_id} '
                     f'반경 {goal.expected_radius_m:.3f} m')
        self.get_logger().info(
            f'정렬 시작: {goal.target_class}, standoff {goal.standoff_m:.3f} m, '
            f'제한 {goal.timeout_sec:.1f} s{where}')

        cp = ControlParams(**{k: self.param(k) for k in vars(ControlParams())})
        sp = SessionParams(**{k: self.param(k) for k in vars(SessionParams())})
        session = AlignSession(goal.standoff_m, goal.timeout_sec, self.now_sec(), cp, sp)
        rate = self.create_rate(self.param('control_rate'))
        fb_every = max(1, round(self.param('control_rate') / self.param('feedback_rate')))
        result = Align.Result()
        k = 0
        try:
            if goal.expected_position.header.frame_id:
                # 정렬 시작 시점에 한 번 base_link로 옮겨 둔다 (interfaces.md 4.1).
                # 첫 검출 전에는 차체를 움직이지 않으므로 이후 프레임과 같은 기준이다.
                xy = self.expected_in_base(goal.expected_position)
                if xy is None:
                    self.publish_cmd(0.0, 0.0)
                    goal_handle.abort()
                    result.success = False
                    result.error_code = TF_ERROR
                    result.final_range_error_m = result.final_bearing_error_rad = float('nan')
                    self.get_logger().info(f'정렬 종료: success=False code={TF_ERROR!r}')
                    return result
                with self.lock:
                    self.expected = (xy, goal.expected_radius_m)
            while rclpy.ok() and not self.shutting_down.is_set():
                if goal_handle.is_cancel_requested:
                    self.publish_cmd(0.0, 0.0)
                    goal_handle.canceled()
                    result.success = False
                    result.final_range_error_m, result.final_bearing_error_rad = \
                        session.last_error
                    self.get_logger().info('정렬 취소됨, 정지')
                    return result

                r = session.tick(self.now_sec(), self.measurement)
                self.publish_cmd(r.vx, r.wz)

                if r.status != RUNNING:
                    result.success = r.status == SUCCEEDED
                    result.error_code = r.error_code
                    result.final_range_error_m = r.e_r
                    result.final_bearing_error_rad = r.e_b
                    if result.success:
                        goal_handle.succeed()
                    else:
                        goal_handle.abort()
                    self.get_logger().info(
                        f'정렬 종료: success={result.success} code={r.error_code!r} '
                        f'e_r={r.e_r:+.4f} m e_b={r.e_b:+.4f} rad')
                    return result

                if k % fb_every == 0:
                    fb = Align.Feedback()
                    fb.target_visible = r.target_visible
                    fb.range_error_m = r.e_r
                    fb.bearing_error_rad = r.e_b
                    goal_handle.publish_feedback(fb)
                    self.get_logger().info(
                        f'[{r.phase}] e_r={r.e_r:+.3f} m e_b={r.e_b:+.3f} rad '
                        f'vx={r.vx:+.3f} wz={r.wz:+.3f}', throttle_duration_sec=1.0)
                k += 1
                rate.sleep()
            # 노드 종료 중: 정지하고 목표 중단
            self.publish_cmd(0.0, 0.0)
            goal_handle.abort()
            result.final_range_error_m, result.final_bearing_error_rad = session.last_error
            self.get_logger().info('노드 종료로 정렬 중단, 정지')
            return result
        except ROSInterruptException:
            return result
        finally:
            with self.lock:
                self.active_target = None
                self.measurement = None
                self.last_uv = None
                self.expected = None
            self.destroy_rate(rate)
            self.idle.set()

    def stop(self):
        self.publish_cmd(0.0, 0.0)


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # 기본 신호 처리기는 Ctrl+C에서 컨텍스트를 먼저 내려 0 속도를 못 보낸다.
    # 직접 KeyboardInterrupt를 받아 0 속도를 보낸 뒤 종료한다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    node = AlignNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        # spin()은 할 일이 없으면 C 함수 안에서 무한정 기다려 Python 신호 처리기가 돌지 못한다.
        # 짧은 timeout으로 돌려야 Ctrl+C / SIGTERM을 받아 0 속도를 보내고 끝낼 수 있다.
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 반드시 0 속도 (설계 문서 2.5).
        # 실행 중인 목표가 있으면 먼저 멈추게 하고 끝날 때까지 잠깐 기다린다.
        # (먼저 0을 보내도 제어 스레드가 그 뒤에 0이 아닌 명령을 보낼 수 있어서)
        node.shutting_down.set()
        deadline = time.monotonic() + 1.0
        while not node.idle.is_set() and time.monotonic() < deadline:
            try:
                executor.spin_once(timeout_sec=0.05)
            except Exception:  # noqa: BLE001
                break
        try:
            node.stop()
        except Exception as e:  # noqa: BLE001  컨텍스트가 이미 내려간 경우
            print(f'[servoing_align] 0 속도 발행 실패: {e}')
        try:
            executor.shutdown(timeout_sec=1.0)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
