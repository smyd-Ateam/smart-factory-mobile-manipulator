"""시험용 /perception/target_pose 발행 (geometry_msgs/PoseStamped, base_link).

enabled true면 rate_hz로 (x, y, z)를 발행한다. stamp는 지금 sim time (이미지를 찍은 시각 대신).
enabled false면 아무것도 발행하지 않는다 (물체가 안 보이는 경우, interfaces.md 3.3).
"""

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from mm_task_manager.run import run


class MockTargetPose(Node):

    def __init__(self):
        super().__init__('mock_target_pose')
        self.enabled = self.declare_parameter('enabled', True).value
        rate = self.declare_parameter('rate_hz', 10.0).value
        self.xyz = (self.declare_parameter('x', 0.25).value,
                    self.declare_parameter('y', 0.0).value,
                    self.declare_parameter('z', 0.02).value)
        self.pub = self.create_publisher(PoseStamped, '/perception/target_pose', 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'enabled {self.enabled}, pose {self.xyz}')

    def _tick(self):
        if not self.enabled:
            return
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.xyz
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)


def main(args=None):
    run(MockTargetPose, args)
