"""베이스 미세 정렬 노드 (3주차: P제어).

입력: /perception/detections, /camera/depth/image_raw, /camera/camera_info
출력: /servoing/cmd_vel

Align 액션 서버(5주차)가 생기기 전까지는 파라미터 enabled, target_class,
standoff_m 으로 동작을 켜고 목표를 준다.
"""
import signal
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray

from visual_servoing.control import ControlParams, compute_cmd, within_tolerance
from visual_servoing.geometry import (decode_depth, depth_at, pixel_to_camera,
                                      range_bearing_error, transform_point)


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class AlignNode(Node):

    def __init__(self):
        super().__init__('servoing_align')

        self.declare_parameter('enabled', True)
        self.declare_parameter('target_class', '')
        self.declare_parameter('standoff_m', 0.25)   # 초기 시험값 (interfaces.md, B 제안)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('score_min', 0.5)
        self.declare_parameter('depth_half_window', 3)
        self.declare_parameter('depth_sync_tolerance', 0.02)   # s
        self.declare_parameter('max_measurement_age', 0.5)     # s, 이보다 오래되면 정지
        self.declare_parameter('control_rate', 20.0)           # Hz
        for name, value in vars(ControlParams()).items():
            self.declare_parameter(name, value)

        self.depth_cache = deque(maxlen=15)   # (stamp_ns, depth 배열)
        self.camera_info = None
        self.measurement = None               # (stamp_ns, e_r, e_b)
        self.pending_detection = None         # 깊이보다 먼저 도착한 검출

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, '/servoing/cmd_vel', 10)
        self.create_subscription(Detection2DArray, '/perception/detections',
                                 self.on_detections, 10)
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self.on_camera_info, qos_profile_sensor_data)

        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self.on_timer)

    def param(self, name):
        return self.get_parameter(name).value

    def control_params(self):
        return ControlParams(**{k: self.param(k) for k in vars(ControlParams())})

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
        if self.pending_detection is not None:
            det_msg, self.pending_detection = self.pending_detection, None
            self.on_detections(det_msg, retry=False)

    def find_depth(self, t_ns):
        tol = self.param('depth_sync_tolerance') * 1e9
        best = None
        for d_ns, frame, depth in self.depth_cache:
            diff = abs(d_ns - t_ns)
            if diff <= tol and (best is None or diff < best[0]):
                best = (diff, frame, depth)
        return None if best is None else (best[1], best[2])

    def pick_detection(self, msg):
        target = self.param('target_class')
        score_min = self.param('score_min')
        best = None
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            if target and hyp.class_id != target:
                continue
            if hyp.score < score_min:
                continue
            if best is None or hyp.score > best.results[0].hypothesis.score:
                best = det
        return best

    def on_detections(self, msg, retry=True):
        if self.camera_info is None:
            self.get_logger().warn('camera_info 대기 중', throttle_duration_sec=5.0)
            return
        det = self.pick_detection(msg)
        if det is None:
            return

        t_ns = stamp_ns(msg.header.stamp)
        found = self.find_depth(t_ns)
        if found is None:
            if retry:
                self.pending_detection = msg   # 깊이가 오면 다시 처리
            else:
                self.get_logger().warn('같은 stamp의 깊이 이미지 없음',
                                       throttle_duration_sec=5.0)
            return
        _, depth = found

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

        e_r, e_b = range_bearing_error(x_b, y_b, self.param('standoff_m'))
        self.measurement = (t_ns, e_r, e_b)

    # ---------- 제어 ----------

    def publish_cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def on_timer(self):
        if not self.param('enabled'):
            return
        if self.measurement is None:
            self.publish_cmd(0.0, 0.0)
            return
        t_ns, e_r, e_b = self.measurement
        age = (self.get_clock().now().nanoseconds - t_ns) / 1e9
        if age > self.param('max_measurement_age'):
            self.get_logger().warn(f'측정값이 오래됨 ({age:.2f} s), 정지',
                                   throttle_duration_sec=2.0)
            self.publish_cmd(0.0, 0.0)
            return

        p = self.control_params()
        vx, wz = compute_cmd(e_r, e_b, p)
        self.publish_cmd(vx, wz)
        self.get_logger().info(
            f'e_r={e_r:+.3f} m  e_b={e_b:+.3f} rad  vx={vx:+.3f}  wz={wz:+.3f}'
            + ('  [허용 범위]' if within_tolerance(e_r, e_b, p) else ''),
            throttle_duration_sec=1.0)

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 반드시 0 속도 (설계 문서 2.5)
        try:
            node.stop()
        except Exception as e:  # noqa: BLE001  컨텍스트가 이미 내려간 경우
            print(f'[servoing_align] 0 속도 발행 실패: {e}')
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
