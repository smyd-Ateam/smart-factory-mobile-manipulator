"""관측 자세 이동 액션 클라이언트 (/arm/observe_pose, project_interfaces/action/ObservePose).

제안 초안 (A, 2026-11-15): 실제 액션 인터페이스는 B 확인 후 확정 예정이라 바뀔 수 있다.
상태머신 흐름(Nav2 도착 → 관측 자세 → Align)에는 아직 연결하지 않았다. B 설계가 나온 뒤 연결한다.

결과는 on_done(success, error_code, detail)로 알린다. Align/Grasp 클라이언트와 같은 규칙:
error_code는 5.3 코드 이름, 목표 거부는 UNKNOWN / rejected, 취소되면 detail canceled.
"""

from action_msgs.msg import GoalStatus
from project_interfaces.action import ObservePose
from rclpy.action import ActionClient

from mm_task_manager import codes

ACTION_NAME = '/arm/observe_pose'
# interfaces.md 5.6. pre_grasp: 근접 정렬 2단계(near 1차 Align → pre_grasp 전환 → 2차 Align) 대비
POSE_NAMES = ('far', 'near', 'pre_grasp')


class ObservePoseClient:

    def __init__(self, node, action_name=ACTION_NAME):
        self._node = node
        self._client = ActionClient(node, ObservePose, action_name)

    def server_is_ready(self):
        return self._client.server_is_ready()

    def wait_for_server(self, timeout_sec=None):
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def send(self, pose_name, on_done):
        self._node.get_logger().info(f'observe_pose goal: {pose_name}')
        future = self._client.send_goal_async(ObservePose.Goal(pose_name=pose_name))
        future.add_done_callback(lambda f: self._on_accepted(f, on_done))

    def _on_accepted(self, future, on_done):
        handle = future.result()
        if handle is None or not handle.accepted:
            on_done(False, codes.UNKNOWN, codes.REJECTED)
            return
        handle.get_result_async().add_done_callback(lambda f: self._on_result(f, on_done))

    def _on_result(self, future, on_done):
        res = future.result()
        detail = codes.CANCELED if res.status == GoalStatus.STATUS_CANCELED else ''
        self._node.get_logger().info(
            f'observe_pose status {res.status} success {res.result.success} '
            f'code {res.result.error_code!r}')
        on_done(bool(res.result.success), res.result.error_code, detail)
