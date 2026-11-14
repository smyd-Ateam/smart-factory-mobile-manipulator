"""시험용 /servoing/align 서버 (project_interfaces/action/Align). 차체를 움직이지 않는다.

results: 호출마다 결과 (리스트, 다 쓰면 마지막 값을 계속 사용)
  success   duration_sec 뒤 success true
  hang      끝나지 않음 (취소는 받아들임)
  그 외     duration_sec 뒤 success false, error_code = 그 문자열 (예: OBJECT_NOT_FOUND)
실행 중에는 /servoing/cmd_vel로 linear.x = servo_vx를 10 Hz 발행.
받은 목표는 goal_log_path(CSV: call, target_class, standoff_m, timeout_sec)에 남긴다.
시간은 wall time 기준.
"""

import os
import time

from geometry_msgs.msg import Twist
from project_interfaces.action import Align
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from mm_task_manager.run import run


class MockAlignServer(Node):

    def __init__(self):
        super().__init__('mock_align_server')
        self.results = list(self.declare_parameter('results', ['success']).value)
        self.duration = self.declare_parameter('duration_sec', 1.0).value
        self.servo_vx = self.declare_parameter('servo_vx', 0.03).value
        self.cmd_pub = self.create_publisher(Twist, '/servoing/cmd_vel', 10)
        self.goal_log = _open_log(self.declare_parameter('goal_log_path', '').value,
                                  'call,target_class,standoff_m,timeout_sec\n')
        self.calls = 0
        self.closing = False
        self.server = ActionServer(
            self, Align, '/servoing/align',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f'results {self.results}')

    def _on_goal(self, goal):
        self.calls += 1
        self.get_logger().info(f'align #{self.calls}: {goal.target_class} '
                               f'standoff {goal.standoff_m:.3f} timeout {goal.timeout_sec:.1f}')
        if self.goal_log is not None:
            self.goal_log.write(f'{self.calls},{goal.target_class},{goal.standoff_m:.6f},'
                                f'{goal.timeout_sec:.6f}\n')
            self.goal_log.flush()
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        mode = self.results[min(self.calls, len(self.results)) - 1]
        start = time.monotonic()
        result = Align.Result()
        while not self.closing:
            if goal_handle.is_cancel_requested:
                self.cmd_pub.publish(Twist())       # 취소되면 속도 0 (interfaces.md 4.1)
                goal_handle.canceled()
                self.get_logger().info('canceled')
                return result
            if mode != 'hang' and time.monotonic() - start >= self.duration:
                break
            msg = Twist()
            msg.linear.x = self.servo_vx
            self.cmd_pub.publish(msg)
            time.sleep(0.1)
        self.cmd_pub.publish(Twist())
        result.success = mode == 'success'
        result.error_code = '' if result.success else mode
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(f'finished ({mode})')
        return result


def _open_log(path, header):
    if not path:
        return None
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    f = open(path, 'w', encoding='utf-8')
    f.write(header)
    return f


def main(args=None):
    run(MockAlignServer, args, multithreaded=True, on_exit=lambda n: setattr(n, 'closing', True))
