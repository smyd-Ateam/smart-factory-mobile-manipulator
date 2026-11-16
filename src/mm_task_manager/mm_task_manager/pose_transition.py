"""정렬(ALIGN) 중 Near→Pre-grasp 관측 자세 전환 판단 (rclpy 없음).

interfaces.md 5.6: 자세 전환 순서 Far → Near(약 0.60 m) → Pre-grasp(약 0.35 m) → Grasp (B 제안).
정렬(D) 중 전환을 상태머신이 어떻게 요청할지(액션 이름 · 형식, 팔이 움직이는 동안 Align 처리)는
D·B 확인 전이라, 상태머신은 "전환 요청" 명령만 내고 노드는 아직 실제로 보내지 않는다.

거리는 로봇 중심 ~ 물체 중심 수평거리 (interfaces.md 5.6), m.
"""

import math

# TODO: PR #1 답변 대기 중, 0.35m 확정 전.
# 출처: 희선님(B) 개인 노션의 Near→Pre-grasp 전환 지점. 값이 정해지면 여기 한 곳만 고친다.
# (Mission.pregrasp_switch_distance_m 기본값이 이 상수를 쓰고, YAML에서 덮어쓸 수도 있다)
NEAR_TO_PREGRASP_THRESHOLD_M = 0.35

# 관측 자세 이름 (interfaces.md 5.6 pre-grasp). 실제 요청 형식은 B 확인 전
PRE_GRASP = 'pre_grasp'


def horizontal_distance(x, y):
    """base_link 기준 target_pose의 (x, y) → 수평거리, m."""
    return math.hypot(x, y)


def needs_pregrasp(distance_m, threshold_m):
    """물체가 기준 거리 아래로 들어왔으면 True. NaN 등 잘못된 값이면 False."""
    return distance_m < threshold_m
