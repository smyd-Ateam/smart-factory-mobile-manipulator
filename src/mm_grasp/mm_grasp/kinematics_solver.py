
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    'joint1', 'joint2', 'joint3', 'joint4', 'joint5'
]


class KinematicsSolver(Node):
    def __init__(self):
        super().__init__('kinematics_solver')

        # 목표 좌표를 ROS2 파라미터로 입력
        self.declare_parameter('target_x', -0.001)
        self.declare_parameter('target_y', -0.002)
        self.declare_parameter('target_z', 0.456)

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.target_z = float(self.get_parameter('target_z').value)

        # IK / FK 서비스 클라이언트
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik'
        )
        self.fk_client = self.create_client(
            GetPositionFK, '/compute_fk'
        )

        for name, client in [
            ('IK', self.ik_client),
            ('FK', self.fk_client)
        ]:
            self.get_logger().info(f'Waiting for {name} service...')
            while rclpy.ok() and not client.wait_for_service(
                timeout_sec=1.0
            ):
                self.get_logger().info(
                    f'{name} service not available yet'
                )

    def run_ik_fk(self, seed_positions=None):
        # 1. IK 요청 생성
        request = GetPositionIK.Request()
        ik = request.ik_request

        ik.group_name = 'dofbot_arm'
        ik.ik_link_name = 'ee_link'

        target = ik.pose_stamped
        target.header.frame_id = 'arm_base_link'
        target.header.stamp = self.get_clock().now().to_msg()

        target.pose.position.x = self.target_x
        target.pose.position.y = self.target_y
        target.pose.position.z = self.target_z

        # 현재 검증된 TCP 자세로 고정
        target.pose.orientation.x = 0.70710678
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 0.70710678

        ik.robot_state = RobotState()
        ik.robot_state.joint_state = JointState()
        ik.robot_state.joint_state.name = JOINT_NAMES
        ik.robot_state.joint_state.position = (
            list(seed_positions)
            if seed_positions is not None
            else [0.0] * 5
        )

        ik.timeout = Duration(seconds=2.0).to_msg()
        ik.avoid_collisions = False

        self.get_logger().info(
            f'Target: ({self.target_x:.3f}, '
            f'{self.target_y:.3f}, '
            f'{self.target_z:.3f}) m'
        )

        # 2. IK 계산
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.exception() is not None:
            self.get_logger().error(
                f'IK service exception: {future.exception()}'
            )
            return

        response = future.result()

        if response is None or response.error_code.val != 1:
            code = response.error_code.val if response else 'no response'
            self.get_logger().error(f'IK FAILED: {code}')
            return

        self.get_logger().info('IK SUCCESS')

        ik_joints = response.solution.joint_state
        joint_map = dict(zip(
            ik_joints.name, ik_joints.position
        ))

        if not all(name in joint_map for name in JOINT_NAMES):
            self.get_logger().error('IK returned incomplete joints')
            return

        for name in JOINT_NAMES:
            self.get_logger().info(
                f'{name}: {joint_map[name]:.6f} rad'
            )

        # 3. IK 결과를 그대로 FK에 전달
        fk_request = GetPositionFK.Request()

        fk_request.header.frame_id = 'arm_base_link'
        fk_request.header.stamp = self.get_clock().now().to_msg()
        fk_request.fk_link_names = ['ee_link']

        fk_request.robot_state = RobotState()
        fk_request.robot_state.joint_state = JointState()
        fk_request.robot_state.joint_state.name = JOINT_NAMES
        fk_request.robot_state.joint_state.position = [
            joint_map[name] for name in JOINT_NAMES
        ]

        fk_future = self.fk_client.call_async(fk_request)
        rclpy.spin_until_future_complete(self, fk_future)

        if fk_future.exception() is not None:
            self.get_logger().error(
                f'FK service exception: {fk_future.exception()}'
            )
            return

        fk_response = fk_future.result()

        if fk_response is None or fk_response.error_code.val != 1:
            code = (
                fk_response.error_code.val
                if fk_response else 'no response'
            )
            self.get_logger().error(f'FK FAILED: {code}')
            return

        if not fk_response.pose_stamped:
            self.get_logger().error('FK returned no pose')
            return

        fk_pose = fk_response.pose_stamped[0].pose
        target_pose = target.pose

        # 4. 위치 오차 계산
        dx = fk_pose.position.x - target_pose.position.x
        dy = fk_pose.position.y - target_pose.position.y
        dz = fk_pose.position.z - target_pose.position.z

        position_error = math.sqrt(dx**2 + dy**2 + dz**2)

        # 5. 자세 오차 계산
        q1 = target_pose.orientation
        q2 = fk_pose.orientation

        dot = abs(
            q1.x * q2.x +
            q1.y * q2.y +
            q1.z * q2.z +
            q1.w * q2.w
        )

        norm1 = math.sqrt(
            q1.x**2 + q1.y**2 + q1.z**2 + q1.w**2
        )
        norm2 = math.sqrt(
            q2.x**2 + q2.y**2 + q2.z**2 + q2.w**2
        )

        dot = min(1.0, max(0.0, dot / (norm1 * norm2)))
        orientation_error = 2.0 * math.acos(dot)

        self.get_logger().info('FK SUCCESS')

        self.get_logger().info(
            f'FK position: '
            f'({fk_pose.position.x:.6f}, '
            f'{fk_pose.position.y:.6f}, '
            f'{fk_pose.position.z:.6f}) m'
        )

        self.get_logger().info(
            f'Position error: {position_error * 1000:.3f} mm'
        )
        self.get_logger().info(
            f'Orientation error: '
            f'{math.degrees(orientation_error):.4f} deg'
        )
        # FK 검증 결과 확인
        if position_error > 0.001 or math.degrees(orientation_error) > 1.0:
            self.get_logger().error('FK validation failed')
            return None
        # IK로 계산한 관절각을 외부에 반환
        return [joint_map[name] for name in JOINT_NAMES]
def main(args=None):
    rclpy.init(args=args)
    node = KinematicsSolver()

    try:
        node.run_ik_fk()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()