"""시험용 /clock 발행 (Gazebo 대신). stop_after_sec(wall) 뒤에 발행을 멈춘다 (S9)."""

import time

from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from mm_task_manager.run import run


class MockClock(Node):

    def __init__(self):
        super().__init__('mock_clock')
        self.rate_hz = self.declare_parameter('rate_hz', 100.0).value
        self.stop_after = self.declare_parameter('stop_after_sec', -1.0).value
        self.pub = self.create_publisher(Clock, '/clock', 10)
        self.start = time.monotonic()
        self.stopped = False
        self.create_timer(1.0 / self.rate_hz, self._publish)   # wall time 타이머

    def _publish(self):
        elapsed = time.monotonic() - self.start
        if 0.0 <= self.stop_after < elapsed:
            if not self.stopped:
                self.stopped = True
                self.get_logger().warn(f'/clock stopped at {elapsed:.2f} s')
            return
        sec = int(elapsed)
        self.pub.publish(Clock(clock=TimeMsg(sec=sec, nanosec=int((elapsed - sec) * 1e9))))


def main(args=None):
    run(MockClock, args)
