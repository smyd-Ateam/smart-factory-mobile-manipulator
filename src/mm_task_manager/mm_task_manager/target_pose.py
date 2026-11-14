"""Grasp에 넘길 /perception/target_pose 선택 (rclpy 없음).

interfaces.md 6장: 정렬 완료 후 받은 값 중 stamp 기준 max_age(1.0 s) 이내.
interfaces.md 3.3: stamp 기준 1 s 넘게 새 값이 없으면 미검출 → 상태머신은 OBJECT_NOT_FOUND / target_pose_stale.
시각은 모두 sim time 초. stamp는 C가 이미지를 찍은 시각.
"""

from dataclasses import dataclass

SEND = 'send'      # 이 값으로 Grasp 목표를 보냄
WAIT = 'wait'      # 아직 쓸 값이 없음, 더 기다림
STALE = 'stale'    # max_age가 지나도록 쓸 값이 없음 → 미검출


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
