"""시험용 /navigate_to_pose 서버. 차체를 움직이지 않는다.

mode:
  succeed             duration_sec 뒤 SUCCEEDED
  aborted             duration_sec 뒤 ABORTED
  reject              목표 거부
  hang                끝나지 않음 (취소는 받아들임)
  hang_ignore_cancel  끝나지 않고, 취소 요청에 응답도 하지 않음
실행 중에는 /nav2/cmd_vel로 linear.x = cmd_vx를 10 Hz 발행 (cmd_stop_after_sec 뒤 중단, -1이면 계속).
끝난 뒤에도 cmd_after_finish_sec 동안 계속 발행 (S11).
시간은 wall time 기준 (/clock 정지 시험에서도 동작하도록).
"""

import time

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future

from mm_task_manager.run import run


class MockNavServer(Node):

    def __init__(self):
        super().__init__('mock_nav_server')
        self.mode = self.declare_parameter('mode', 'succeed').value
        self.duration = self.declare_parameter('duration_sec', 3.0).value
        self.cmd_vx = self.declare_parameter('cmd_vx', 0.11).value
        self.cmd_stop_after = self.declare_parameter('cmd_stop_after_sec', -1.0).value
        # 목표가 끝난 뒤에도 /nav2/cmd_vel을 이 시간(wall) 동안 계속 발행 (S11)
        self.cmd_after_finish = self.declare_parameter('cmd_after_finish_sec', 0.0).value
        self.extra_until = 0.0
        self.create_timer(0.1, self._extra_cmd)
        self.cmd_pub = self.create_publisher(Twist, '/nav2/cmd_vel', 10)
        self.closing = False
        self.never = Future()       # 완료되지 않는 Future (취소 무응답용)
        self.goal_count = 0
        self.server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f'mode {self.mode}')

    def _extra_cmd(self):
        if time.monotonic() < self.extra_until:
            msg = Twist()
            msg.linear.x = self.cmd_vx
            self.cmd_pub.publish(msg)

    def _on_goal(self, goal):
        self.goal_count += 1
        p = goal.pose.pose.position
        self.get_logger().info(f'goal #{self.goal_count} received ({p.x:.2f}, {p.y:.2f})')
        return GoalResponse.REJECT if self.mode == 'reject' else GoalResponse.ACCEPT

    async def _on_cancel(self, goal_handle):
        self.get_logger().info('cancel requested')
        if self.mode == 'hang_ignore_cancel':
            await self.never                     # 응답을 보내지 않음
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        start = time.monotonic()
        result = NavigateToPose.Result()
        while not self.closing:
            elapsed = time.monotonic() - start
            if goal_handle.is_cancel_requested and self.mode != 'hang_ignore_cancel':
                goal_handle.canceled()
                self.get_logger().info(f'canceled at {elapsed:.2f} s')
                return result
            if self.mode in ('succeed', 'aborted') and elapsed >= self.duration:
                break
            if self.cmd_stop_after < 0.0 or elapsed < self.cmd_stop_after:
                msg = Twist()
                msg.linear.x = self.cmd_vx
                self.cmd_pub.publish(msg)
            time.sleep(0.1)
        self.extra_until = time.monotonic() + self.cmd_after_finish
        if self.mode == 'aborted':
            goal_handle.abort()
        else:
            goal_handle.succeed()
        self.get_logger().info(f'finished ({self.mode}) at {time.monotonic() - start:.2f} s')
        return result


def main(args=None):
    run(MockNavServer, args, multithreaded=True, on_exit=lambda n: setattr(n, 'closing', True))
