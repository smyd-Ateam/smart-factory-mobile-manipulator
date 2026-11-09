"""/cmd_vel 기록 (CSV: wall_time, sim_time, vx, wz). 시나리오 결과를 숫자로 확인하기 위함."""

import os
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node

from mm_task_manager.run import run


class CmdRecorder(Node):

    def __init__(self):
        super().__init__('cmd_recorder')
        path = os.path.expanduser(self.declare_parameter('csv_path', 'cmd_vel.csv').value)
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self.f = open(path, 'w', encoding='utf-8')
        self.f.write('wall_time,sim_time,vx,wz\n')
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)

    def _on_cmd(self, msg):
        sim = self.get_clock().now().nanoseconds * 1e-9
        self.f.write(f'{time.time():.3f},{sim:.3f},{msg.linear.x:.3f},{msg.angular.z:.3f}\n')
        self.f.flush()


def main(args=None):
    run(CmdRecorder, args, on_exit=lambda n: n.f.close())
