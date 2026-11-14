"""시험용 /grasp/execute 서버 (project_interfaces/action/Grasp). 팔을 움직이지 않는다.

results: 호출마다 결과 (리스트, 다 쓰면 마지막 값을 계속 사용)
  success   duration_sec 뒤 success true
  hang      끝나지 않음 (취소는 받아들임)
  그 외     duration_sec 뒤 success false, error_code = 그 문자열 (예: IK_FAILED)
받은 목표는 goal_log_path(CSV: call, target_class, frame_id, x, y, z, age_sec)에 남긴다.
age_sec = 받은 시각(sim time) − target_pose.header.stamp.
"""

import time

from project_interfaces.action import Grasp
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from mm_task_manager.mocks.mock_align_server import _open_log
from mm_task_manager.run import run


class MockGraspServer(Node):

    def __init__(self):
        super().__init__('mock_grasp_server')
        self.results = list(self.declare_parameter('results', ['success']).value)
        self.duration = self.declare_parameter('duration_sec', 1.0).value
        self.goal_log = _open_log(self.declare_parameter('goal_log_path', '').value,
                                  'call,target_class,frame_id,x,y,z,age_sec\n')
        self.calls = 0
        self.closing = False
        self.server = ActionServer(
            self, Grasp, '/grasp/execute',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f'results {self.results}')

    def _on_goal(self, goal):
        self.calls += 1
        p = goal.target_pose
        stamp = p.header.stamp.sec + p.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        self.get_logger().info(f'grasp #{self.calls}: {goal.target_class} '
                               f'pose ({p.pose.position.x:.3f}, {p.pose.position.y:.3f}, '
                               f'{p.pose.position.z:.3f}) {p.header.frame_id} age {age:.3f} s')
        if self.goal_log is not None:
            self.goal_log.write(f'{self.calls},{goal.target_class},{p.header.frame_id},'
                                f'{p.pose.position.x:.6f},{p.pose.position.y:.6f},'
                                f'{p.pose.position.z:.6f},{age:.6f}\n')
            self.goal_log.flush()
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        mode = self.results[min(self.calls, len(self.results)) - 1]
        start = time.monotonic()
        result = Grasp.Result()
        while not self.closing:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            if mode != 'hang' and time.monotonic() - start >= self.duration:
                break
            time.sleep(0.1)
        result.success = mode == 'success'
        result.error_code = '' if result.success else mode
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(f'finished ({mode})')
        return result


def main(args=None):
    run(MockGraspServer, args, multithreaded=True, on_exit=lambda n: setattr(n, 'closing', True))
