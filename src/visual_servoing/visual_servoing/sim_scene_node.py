"""Gazebo 시험(align_sim.sdf) 보조 노드.

- boundingbox 카메라 박스(/servoing/sim_boxes, label 번호)를 클래스 이름으로 바꿔
  /perception/detections 로 낸다. YOLO 대신 쓰는 정답 검출이다 (gt_detections:=false면 끔).
- Gazebo 차체 실제 위치(/servoing/sim_base_pose)로 base_link 기준 물체 위치를 계산해
  /servoing/sim_object_in_base 로 낸다 (시험 판정용).
- /servoing/sim_reset (std_srvs/Trigger): 차체를 robot_start 파라미터 위치로 옮긴다.

파라미터·서비스 모양은 mock_scene_node와 같게 해서 mock_trials를 그대로 쓴다
(mock_trials --ros-args -p scene:=sim).
"""
import math
import subprocess

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection2DArray

from visual_servoing.mock_model import object_in_base, true_error
from visual_servoing.sim_model import label_to_class, set_pose_request, yaw_from_quat

OPTICAL_FRAME = 'camera_optical_frame'
SET_POSE_TIMEOUT_MS = 3000


class SimSceneNode(Node):

    def __init__(self):
        super().__init__('servoing_sim_scene')
        self.declare_parameter('world', 'align_sim')
        self.declare_parameter('model', 'mm_test_base')
        self.declare_parameter('object_xyz', [0.60, 0.08, 0.0])   # align_sim.sdf pin_target 위치
        self.declare_parameter('robot_start', [0.0, 0.0, 0.25])    # x, y, yaw
        self.declare_parameter('base_height', 0.05)
        self.declare_parameter('standoff_m', 0.25)                 # 실제 오차 로그용
        self.declare_parameter('base_frame', 'base_link')
        # false면 정답 박스를 내보내지 않는다 (YOLO 검출 노드가 /perception/detections 발행)
        self.declare_parameter('gt_detections', True)

        self.pose = None   # (x, y, yaw) world 기준
        self.det_pub = self.create_publisher(Detection2DArray, '/perception/detections', 10)
        self.truth_pub = self.create_publisher(PointStamped, '/servoing/sim_object_in_base', 10)
        self.cmd_pub = self.create_publisher(Twist, '/servoing/cmd_vel', 10)
        self.create_subscription(Detection2DArray, '/servoing/sim_boxes', self.on_boxes, 10)
        self.create_subscription(TFMessage, '/servoing/sim_base_pose', self.on_pose, 10)
        self.create_service(Trigger, '/servoing/sim_reset', self.on_reset)
        self.create_timer(1.0, self.on_log)

    def p(self, name):
        return self.get_parameter(name).value

    def on_boxes(self, msg):
        if not self.p('gt_detections'):
            return
        out = Detection2DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = OPTICAL_FRAME
        for det in msg.detections:
            if not det.results:
                continue
            name = label_to_class(det.results[0].hypothesis.class_id)
            if name is None:
                continue
            det.header = out.header
            det.results[0].hypothesis.class_id = name
            det.results[0].hypothesis.score = 1.0
            out.detections.append(det)
        self.det_pub.publish(out)

    def on_pose(self, msg):
        model = self.p('model')
        for t in msg.transforms:
            if t.child_frame_id != model:
                continue
            r = t.transform.rotation
            self.pose = (t.transform.translation.x, t.transform.translation.y,
                         yaw_from_quat(r.x, r.y, r.z, r.w))
            truth = PointStamped()
            truth.header.stamp = t.header.stamp
            truth.header.frame_id = self.p('base_frame')
            truth.point.x, truth.point.y, truth.point.z = self.p_base()
            self.truth_pub.publish(truth)
            return

    def p_base(self):
        return object_in_base(*self.pose, *self.p('object_xyz'), self.p('base_height'))

    def on_reset(self, request, response):
        self.cmd_pub.publish(Twist())
        x, y, yaw = self.p('robot_start')
        req = set_pose_request(self.p('model'), x, y, self.p('base_height'), yaw)
        cmd = ['ign', 'service', '-s', f'/world/{self.p("world")}/set_pose',
               '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
               '--timeout', str(SET_POSE_TIMEOUT_MS), '--req', req]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True,
                                 timeout=SET_POSE_TIMEOUT_MS / 1000 + 2)
        except (OSError, subprocess.TimeoutExpired) as e:
            response.success, response.message = False, f'set_pose 실패: {e}'
            self.get_logger().error(response.message)
            return response
        response.success = out.returncode == 0 and 'true' in out.stdout
        response.message = f'pose=({x:.3f}, {y:.3f}, {yaw:.3f}) {out.stdout.strip()}'
        if not response.success:
            self.get_logger().error(f'set_pose 실패: {out.stdout} {out.stderr}')
        return response

    def on_log(self):
        if self.pose is None:
            self.get_logger().warn('Gazebo 차체 위치를 아직 못 받음 (/servoing/sim_base_pose)',
                                   throttle_duration_sec=5.0)
            return
        e_r, e_b = true_error(self.p_base(), self.p('standoff_m'))
        x, y, yaw = self.pose
        self.get_logger().info(
            f'[실제] e_r={e_r:+.3f} m  e_b={e_b:+.3f} rad  '
            f'pose=({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f}°)')


def main(args=None):
    rclpy.init(args=args)
    node = SimSceneNode()
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
