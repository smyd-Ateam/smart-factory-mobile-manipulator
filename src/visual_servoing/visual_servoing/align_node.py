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
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray

from visual_servoing.control import ControlParams
from visual_servoing.geometry import decode_depth, depth_at, pixel_to_camera, transform_point
from visual_servoing.session import RUNNING, SUCCEEDED, AlignSession, SessionParams


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class AlignNode(Node):

    def __init__(self):
        super().__init__('servoing_align')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('score_min', 0.5)
        self.declare_parameter('depth_half_window', 3)
        self.declare_parameter('depth_sync_tolerance', 0.02)   # s
        self.declare_parameter('control_rate', 20.0)           # Hz
        self.declare_parameter('feedback_rate', 5.0)           # Hz
        for name, value in vars(ControlParams()).items():
            self.declare_parameter(name, value)
        for name, value in vars(SessionParams()).items():
            self.declare_parameter(name, value)

        self.depth_cache = deque(maxlen=15)   # (stamp_ns, frame_id, depth 배열)
        self.camera_info = None
        self.pending_detection = None         # 깊이보다 먼저 도착한 검출
        self.lock = threading.Lock()
        self.active_target = None             # 실행 중인 목표의 target_class
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
                                 self.on_depth, qos_profile_sensor_data, callback_group=group)
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self.on_camera_info, qos_profile_sensor_data,
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

    def pick_detection(self, msg, target):
        score_min = self.param('score_min')
        best = None
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            if hyp.class_id != target or hyp.score < score_min:
                continue
            if best is None or hyp.score > best.results[0].hypothesis.score:
                best = det
        return best

    def on_detections(self, msg, retry=True):
        target = self.active_target
        if target is None:
            return   # 실행 중인 목표가 없으면 계산하지 않음
        if self.camera_info is None:
            self.get_logger().warn('camera_info 대기 중', throttle_duration_sec=5.0)
            return
        det = self.pick_detection(msg, target)
        if det is None:
            return

        t_ns = stamp_ns(msg.header.stamp)
        depth = self.find_depth(t_ns)
        if depth is None:
            if retry:
                self.pending_detection = msg   # 깊이가 오면 다시 처리
            else:
                self.get_logger().warn('같은 stamp의 깊이 이미지 없음', throttle_duration_sec=5.0)
            return

        u, v = det.bbox.center.position.x, det.bbox.center.position.y
        half = max(1, min(self.param('depth_half_window'),
                          int(det.bbox.size_x / 4), int(det.bbox.size_y / 4)))
        z = depth_at(depth, u, v, half)
        if z is None:
            self.get_logger().warn('bbox 중심에 유효한 깊이 없음', throttle_duration_sec=5.0)
            return

        k = self.camera_info.k
        p_cam = pixel_to_camera(u, v, z, k[0], k[4], k[2], k[5])
        frame = msg.header.frame_id or self.camera_info.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(
                self.param('base_frame'), frame, Time(nanoseconds=t_ns),
                timeout=Duration(seconds=0.05))
        except TransformException as e:
            self.get_logger().warn(f'TF 실패: {e}', throttle_duration_sec=5.0)
            return
        tr, q = tf.transform.translation, tf.transform.rotation
        x_b, y_b, _ = transform_point(p_cam, (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
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
        self.get_logger().info(
            f'정렬 시작: {goal.target_class}, standoff {goal.standoff_m:.3f} m, '
            f'제한 {goal.timeout_sec:.1f} s')

        cp = ControlParams(**{k: self.param(k) for k in vars(ControlParams())})
        sp = SessionParams(**{k: self.param(k) for k in vars(SessionParams())})
        session = AlignSession(goal.standoff_m, goal.timeout_sec, self.now_sec(), cp, sp)
        rate = self.create_rate(self.param('control_rate'))
        fb_every = max(1, round(self.param('control_rate') / self.param('feedback_rate')))
        result = Align.Result()
        k = 0
        try:
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
