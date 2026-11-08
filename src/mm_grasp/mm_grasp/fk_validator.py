
import math

import rclpy
from rclpy.node import Node

from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


class FKValidator(Node):
    def __init__(self):
        super().__init__('fk_validator')

        self.client = self.create_client(
            GetPositionFK,
            '/compute_fk'
        )

        self.get_logger().info('Waiting for /compute_fk...')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('FK service not available yet')

    def run_fk(self):
        request = GetPositionFK.Request()

        # FK 결과를 표현할 기준 좌표계
        request.header.frame_id = 'arm_base_link'
        request.header.stamp = self.get_clock().now().to_msg()

        # FK를 계산할 링크 (TCP)
        request.fk_link_names = ['ee_link']

        # 앞서 IK로 계산한 관절각
        joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5'
        ]

        joint_positions = [
            -0.1963,
            0.0650,
            -0.1298,
            0.0649,
            0.1963
        ]

        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = joint_names
        request.robot_state.joint_state.position = joint_positions

        self.get_logger().info('Requesting FK solution...')

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        if response is None:
            self.get_logger().error('FK service call failed')
            return

        if response.error_code.val != 1:
            self.get_logger().error(
                f'FK FAILED: error_code={response.error_code.val}'
            )
            return

        if not response.pose_stamped:
            self.get_logger().error('FK returned no pose')
            return

        pose = response.pose_stamped[0].pose

        # 기존 IK 목표 위치
        target_x = -0.001
        target_y = -0.002
        target_z = 0.456

        dx = pose.position.x - target_x
        dy = pose.position.y - target_y
        dz = pose.position.z - target_z

        position_error = math.sqrt(
            dx**2 + dy**2 + dz**2
        )

        # 기존 IK 목표 자세 (xyzw)
        target_q = [0.70710678, 0.0, 0.0, 0.70710678]
        result_q = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]

        # 쿼터니언의 부호가 반대여도 동일한 회전
        dot = abs(sum(
            a * b for a, b in zip(target_q, result_q)
        ))
        dot = min(1.0, max(-1.0, dot))
        orientation_error = 2.0 * math.acos(dot)

        self.get_logger().info('FK SUCCESS')

        self.get_logger().info(
            f'FK position: '
            f'({pose.position.x:.6f}, '
            f'{pose.position.y:.6f}, '
            f'{pose.position.z:.6f}) m'
        )

        self.get_logger().info(
            f'Position error: {position_error * 1000:.3f} mm'
        )

        self.get_logger().info(
            f'Orientation error: '
            f'{math.degrees(orientation_error):.4f} deg'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FKValidator()

    try:
        node.run_fk()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()