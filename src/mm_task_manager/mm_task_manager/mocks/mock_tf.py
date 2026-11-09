"""시험용 TF: map→odom (항등, stop_after_sec 뒤 발행 중단), odom→base_footprint (고정 위치),
base_footprint→base_link (항등). 시각은 sim time. 차체는 움직이지 않는다.
"""

import math

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from mm_task_manager.run import run


def _tf(stamp, parent, child, x=0.0, y=0.0, yaw=0.0):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.rotation.z = math.sin(yaw / 2.0)
    t.transform.rotation.w = math.cos(yaw / 2.0)
    return t


class MockTf(Node):

    def __init__(self):
        super().__init__('mock_tf')
        self.x = self.declare_parameter('x', 0.0).value
        self.y = self.declare_parameter('y', 0.0).value
        self.yaw = self.declare_parameter('yaw', 0.0).value
        self.stop_after = self.declare_parameter('map_odom_stop_after_sec', -1.0).value
        self.stopped = False
        self.br = TransformBroadcaster(self)
        self.create_timer(0.1, self._publish)

    def _publish(self):
        now = self.get_clock().now()
        sec = now.nanoseconds * 1e-9
        stamp = now.to_msg()
        tfs = [_tf(stamp, 'odom', 'base_footprint', self.x, self.y, self.yaw),
               _tf(stamp, 'base_footprint', 'base_link')]
        if 0.0 <= self.stop_after < sec:
            if not self.stopped:
                self.stopped = True
                self.get_logger().warn(f'map->odom stopped at sim {sec:.2f} s')
        else:
            tfs.append(_tf(stamp, 'map', 'odom'))
        self.br.sendTransform(tfs)


def main(args=None):
    run(MockTf, args)
