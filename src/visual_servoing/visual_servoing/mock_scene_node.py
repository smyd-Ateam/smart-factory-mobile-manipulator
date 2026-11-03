"""모의 차체 + 모의 카메라 + 모의 검출 노드.

Gazebo 카메라와 YOLO가 붙기 전까지 정렬 노드를 폐루프로 시험하기 위한 것이다.
/servoing/cmd_vel 을 받아 차체를 적분하고, 물체가 보이는 위치를 계산해
interfaces.md 와 같은 토픽으로 발행한다.

발행: /perception/detections, /camera/depth/image_raw, /camera/camera_info,
      TF base_link → camera_optical_frame (static)
"""
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from visual_servoing.geometry import matrix_to_quat, optical_rotation_in_base
from visual_servoing.mock_model import (CameraMount, object_in_base, observe,
                                        step_unicycle, true_error)

OPTICAL_FRAME = 'camera_optical_frame'


class MockSceneNode(Node):

    def __init__(self):
        super().__init__('servoing_mock_scene')

        self.declare_parameter('target_class', 'pin')
        self.declare_parameter('object_xyz', [0.60, 0.08, 0.01])   # odom 좌표 (m)
        self.declare_parameter('object_size', [0.05, 0.02])        # bbox 계산용 폭, 높이 (m)
        self.declare_parameter('robot_start', [0.0, 0.0, 0.25])    # x, y, yaw
        self.declare_parameter('base_height', 0.05)                # base_link 높이 (mm_description)
        self.declare_parameter('standoff_m', 0.30)                 # 실제 오차 로그용
        self.declare_parameter('camera_rate', 10.0)                # Hz
        self.declare_parameter('delay_sec', 0.0)                   # 측정 지연
        self.declare_parameter('noise_px', 0.0)
        self.declare_parameter('noise_depth', 0.0)                 # m
        self.declare_parameter('base_frame', 'base_link')

        self.mount = CameraMount()
        self.obj = list(self.get_parameter('object_xyz').value)
        self.obj_size = list(self.get_parameter('object_size').value)
        self.x, self.y, self.yaw = self.get_parameter('robot_start').value
        self.cmd = (0.0, 0.0)
        self.rng = np.random.default_rng(0)
        self.pending = deque()   # (공개 시각 ns, detections, depth, info)

        self.det_pub = self.create_publisher(Detection2DArray, '/perception/detections', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.create_subscription(Twist, '/servoing/cmd_vel', self.on_cmd, 10)

        self.publish_static_tf()

        self.sim_dt = 0.01
        self.create_timer(self.sim_dt, self.on_sim)
        self.create_timer(1.0 / self.get_parameter('camera_rate').value, self.on_capture)
        self.create_timer(0.01, self.on_release)
        self.create_timer(1.0, self.on_log)

    def on_cmd(self, msg):
        # Gazebo DiffDrive와 같이 마지막 명령을 계속 유지한다.
        self.cmd = (msg.linear.x, msg.angular.z)

    def on_sim(self):
        self.x, self.y, self.yaw = step_unicycle(
            self.x, self.y, self.yaw, self.cmd[0], self.cmd[1], self.sim_dt)

    def p_base(self):
        return object_in_base(self.x, self.y, self.yaw, *self.obj,
                              self.get_parameter('base_height').value)

    def publish_static_tf(self):
        self.static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('base_frame').value
        t.child_frame_id = OPTICAL_FRAME
        t.transform.translation.x = self.mount.x
        t.transform.translation.z = self.mount.z
        qx, qy, qz, qw = matrix_to_quat(optical_rotation_in_base(self.mount.pitch))
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.static_tf.sendTransform(t)

    def on_capture(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        m = self.mount

        dets = Detection2DArray()
        dets.header.stamp = stamp
        dets.header.frame_id = OPTICAL_FRAME

        depth = np.full((m.height, m.width), np.inf, dtype=np.float32)

        obs = observe(self.p_base(), m, *self.obj_size)
        if obs is not None:
            u, v, z, bw, bh = obs
            npx = self.get_parameter('noise_px').value
            nz = self.get_parameter('noise_depth').value
            u += self.rng.normal(0, npx) if npx > 0 else 0.0
            v += self.rng.normal(0, npx) if npx > 0 else 0.0
            z_meas = z + (self.rng.normal(0, nz) if nz > 0 else 0.0)

            u0, u1 = max(0, int(u - bw / 2)), min(m.width, int(u + bw / 2) + 1)
            v0, v1 = max(0, int(v - bh / 2)), min(m.height, int(v + bh / 2) + 1)
            depth[v0:v1, u0:u1] = z_meas

            det = Detection2D()
            det.header = dets.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.get_parameter('target_class').value
            hyp.hypothesis.score = 0.9
            det.results.append(hyp)
            det.bbox.center.position.x = float(u)
            det.bbox.center.position.y = float(v)
            det.bbox.size_x = float(bw)
            det.bbox.size_y = float(bh)
            dets.detections.append(det)

        img = Image()
        img.header = dets.header
        img.height, img.width = m.height, m.width
        img.encoding = '32FC1'
        img.is_bigendian = 0
        img.step = m.width * 4
        img.data = depth.tobytes()

        info = CameraInfo()
        info.header = dets.header
        info.height, info.width = m.height, m.width
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [m.fx, 0.0, m.cx, 0.0, m.fy, m.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [m.fx, 0.0, m.cx, 0.0, 0.0, m.fy, m.cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        release = now.nanoseconds + int(self.get_parameter('delay_sec').value * 1e9)
        self.pending.append((release, dets, img, info))

    def on_release(self):
        now = self.get_clock().now().nanoseconds
        while self.pending and self.pending[0][0] <= now:
            _, dets, img, info = self.pending.popleft()
            self.info_pub.publish(info)
            self.depth_pub.publish(img)
            self.det_pub.publish(dets)

    def on_log(self):
        e_r, e_b = true_error(self.p_base(), self.get_parameter('standoff_m').value)
        self.get_logger().info(
            f'[실제] e_r={e_r:+.3f} m  e_b={e_b:+.3f} rad  '
            f'pose=({self.x:.3f}, {self.y:.3f}, {math.degrees(self.yaw):.1f}°)')


def main(args=None):
    rclpy.init(args=args)
    node = MockSceneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
