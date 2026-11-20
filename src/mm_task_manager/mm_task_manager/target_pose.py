"""Grasp에 넘길 /perception/target_pose 선택 (rclpy 없음).

interfaces.md 6장: 정렬 완료 후 받은 값 중 stamp 기준 max_age(1.0 s) 이내.
interfaces.md 3.3: stamp 기준 1 s 넘게 새 값이 없으면 미검출 → 상태머신은 OBJECT_NOT_FOUND / target_pose_stale.
시각은 모두 sim time 초. stamp는 C가 이미지를 찍은 시각.

받는 쪽 안전장치 (C 제안, PR #1 댓글 2026-11-18 · 11-20): target_pose_reject_reason이 이유를 돌려준
메시지는 노드가 아예 무시한다 (Grasp 목표 후보에도, Near→Pre-grasp 거리 판단에도 안 씀).
"""

import math
from dataclasses import dataclass

TARGET_POSE_FRAME = 'base_link'   # interfaces.md 3장: /perception/target_pose는 base_link 기준

SEND = 'send'      # 이 값으로 Grasp 목표를 보냄
WAIT = 'wait'      # 아직 쓸 값이 없음, 더 기다림
STALE = 'stale'    # max_age가 지나도록 쓸 값이 없음 → 미검출


# 걸러진 이유 (노드 경고 로그에 그대로 남김)
REJECT_FRAME = 'frame_id'              # frame_id가 정확히 base_link가 아님
REJECT_NON_FINITE = 'non_finite'       # x, y, z 중 NaN · inf
REJECT_X_NOT_POSITIVE = 'x_not_positive'   # x <= 0 (차체 뒤쪽)
REJECT_X_ABOVE_MAX = 'x_above_max'     # x > max_x (관측 범위보다 멂, 깊이 오류)
REJECT_X_JUMP = 'x_jump'               # 직전에 통과한 x 대비 |Δx| >= max_jump (깊이 오류)


@dataclass(frozen=True)
class GraspPoseRequest:
    token: int
    since: float          # 정렬이 끝난 시각 (이보다 먼저 찍힌 값은 정렬 중 값이라 안 씀)
    max_age_sec: float

    def check(self, latest_stamp, now):
        if (latest_stamp is not None and latest_stamp >= self.since
                and now - latest_stamp <= self.max_age_sec):
            return SEND
        if now - self.since > self.max_age_sec:
            return STALE
        return WAIT


def target_pose_reject_reason(frame_id, x, y, z, max_x=math.inf, prev_x=None, max_jump=math.inf):
    """/perception/target_pose를 거를 이유 (REJECT_*), 써도 되면 None. 앞에서부터 처음 걸린 것 하나.

    - frame_id가 정확히 'base_link'가 아니면 거름 (변환하지 않음)
    - x, y, z 중 NaN · inf가 있으면 거름
    - x <= 0이면 거름: 카메라가 항상 앞을 봐서 차체 뒤쪽 물체는 나올 수 없음
    - x > max_x면 거름 (경계 max_x 자체는 통과)
    - prev_x(직전에 통과한 x)가 있고 |x - prev_x| >= max_jump면 거름 (양방향). prev_x가 없으면 안 봄
    """
    if frame_id != TARGET_POSE_FRAME:
        return REJECT_FRAME
    if not all(math.isfinite(v) for v in (x, y, z)):
        return REJECT_NON_FINITE
    if x <= 0.0:
        return REJECT_X_NOT_POSITIVE
    if x > max_x:
        return REJECT_X_ABOVE_MAX
    if prev_x is not None and abs(x - prev_x) >= max_jump:
        return REJECT_X_JUMP
    return None


def is_valid_target_pose(frame_id, x, y, z, max_x=math.inf, prev_x=None, max_jump=math.inf):
    """/perception/target_pose를 써도 되면 True (target_pose_reject_reason이 None)."""
    return target_pose_reject_reason(frame_id, x, y, z, max_x, prev_x, max_jump) is None


class TargetPoseFilter:
    """노드용: 직전에 통과시킨 x를 기억하며 거른다. 걸러진 메시지는 기준(prev_x)을 바꾸지 않는다.

    reset_sec: 새 메시지 stamp가 마지막으로 통과한 메시지 stamp보다 reset_sec 넘게 뒤(또는 거꾸로)면
    직전 값을 지우고 첫 메시지처럼 본다. 물체를 놓쳤다가 다시 잡았을 때 x가 max_jump 넘게 달라져 있어도
    계속 x_jump로 걸리지 않게 하기 위함. None이거나 stamp를 안 주면 초기화하지 않는다.
    """

    def __init__(self, max_x, max_jump, reset_sec=None):
        self.max_x = max_x
        self.max_jump = max_jump
        self.reset_sec = reset_sec
        self.prev_x = None
        self.prev_stamp = None

    def _expired(self, stamp):
        if self.reset_sec is None or stamp is None or self.prev_stamp is None:
            return False
        dt = stamp - self.prev_stamp
        return dt < 0.0 or dt > self.reset_sec

    def check(self, frame_id, x, y, z, stamp=None):
        prev_x = None if self._expired(stamp) else self.prev_x
        reason = target_pose_reject_reason(frame_id, x, y, z, self.max_x, prev_x, self.max_jump)
        if reason is None:
            self.prev_x = x
            self.prev_stamp = stamp
        return reason
