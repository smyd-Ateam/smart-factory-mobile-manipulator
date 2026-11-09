"""시험용 /cmd_vel 발행자 (S10: 이전 실행 노드가 남은 상황 흉내). enabled가 false면 발행자를 만들지 않는다."""

from geometry_msgs.msg import Twist
from rclpy.node import Node

from mm_task_manager.run import run


class MockCmdPublisher(Node):

    def __init__(self):
        super().__init__('mock_cmd_publisher')
        if not self.declare_parameter('enabled', False).value:
            return
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, lambda: self.pub.publish(Twist()))
        self.get_logger().info('publishing zero on /cmd_vel (conflict test)')


def main(args=None):
    run(MockCmdPublisher, args)
