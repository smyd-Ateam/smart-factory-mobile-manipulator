"""/cmd_vel과 /task_manager/state 기록 (CSV: wall_time, sim_time, topic, vx, wz, state).

시나리오 결과를 숫자로 확인하기 위함. topic은 cmd_vel 또는 state.
"""

import os
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from mm_task_manager.run import run

HEADER = 'wall_time,sim_time,topic,vx,wz,state\n'


class CmdRecorder(Node):

    def __init__(self):
        super().__init__('cmd_recorder')
        path = os.path.expanduser(self.declare_parameter('csv_path', 'cmd_vel.csv').value)
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self.f = open(path, 'w', encoding='utf-8')
        self.f.write(HEADER)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/task_manager/state', self._on_state, latched)

    def _row(self, topic, vx=0.0, wz=0.0, state=''):
        sim = self.get_clock().now().nanoseconds * 1e-9
        self.f.write(f'{time.time():.3f},{sim:.3f},{topic},{vx:.3f},{wz:.3f},{state}\n')
        self.f.flush()

    def _on_cmd(self, msg):
        self._row('cmd_vel', msg.linear.x, msg.angular.z)

    def _on_state(self, msg):
        self._row('state', state=msg.data)


def main(args=None):
    run(CmdRecorder, args, on_exit=lambda n: n.f.close())
