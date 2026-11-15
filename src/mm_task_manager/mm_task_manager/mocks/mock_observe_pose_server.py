"""시험용 /arm/observe_pose 서버 (project_interfaces/action/ObservePose, 제안 초안). 팔을 움직이지 않는다.

pose_name을 받으면 duration_sec(기본 0.5 s, wall time) 뒤 success true를 돌려준다.
받은 목표는 goal_log_path(CSV: call, pose_name)에 남긴다.
실제 인터페이스는 B 확인 후 확정 예정 (observe_pose_client.py 참고).
"""

import time

from project_interfaces.action import ObservePose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from mm_task_manager.mocks.mock_align_server import _open_log
from mm_task_manager.run import run


class MockObservePoseServer(Node):

    def __init__(self):
        super().__init__('mock_observe_pose_server')
        self.duration = self.declare_parameter('duration_sec', 0.5).value
        self.goal_log = _open_log(self.declare_parameter('goal_log_path', '').value,
                                  'call,pose_name\n')
        self.calls = 0
        self.closing = False
        self.server = ActionServer(
            self, ObservePose, '/arm/observe_pose',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f'duration {self.duration:.1f} s')

    def _on_goal(self, goal):
        self.calls += 1
        self.get_logger().info(f'observe_pose #{self.calls}: {goal.pose_name}')
        if self.goal_log is not None:
            self.goal_log.write(f'{self.calls},{goal.pose_name}\n')
            self.goal_log.flush()
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        start = time.monotonic()
        result = ObservePose.Result()
        while not self.closing and time.monotonic() - start < self.duration:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            time.sleep(0.05)
        result.success = True
        result.error_code = ''
        goal_handle.succeed()
        self.get_logger().info(f'finished ({goal_handle.request.pose_name})')
        return result


def main(args=None):
    run(MockObservePoseServer, args, multithreaded=True,
        on_exit=lambda n: setattr(n, 'closing', True))
