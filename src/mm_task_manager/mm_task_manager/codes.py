"""실패 코드 (docs/interfaces.md 5.3 + 임시 추가 코드)와 세부 원인."""

# interfaces.md 5.3
NAV_FAILED = 'NAV_FAILED'
OBJECT_NOT_FOUND = 'OBJECT_NOT_FOUND'
ALIGN_TIMEOUT = 'ALIGN_TIMEOUT'
IK_FAILED = 'IK_FAILED'
GRASP_MISS = 'GRASP_MISS'
GRASP_SLIP = 'GRASP_SLIP'
UNKNOWN = 'UNKNOWN'

# 임시 추가 (나중에 interfaces.md 5장에 반영)
NAV_NOT_READY = 'NAV_NOT_READY'

ALL_CODES = frozenset({
    NAV_FAILED, OBJECT_NOT_FOUND, ALIGN_TIMEOUT, IK_FAILED, GRASP_MISS, GRASP_SLIP,
    UNKNOWN, NAV_NOT_READY,
})

# 결과 기록의 detail (세부 원인)
ARRIVAL_CHECK = 'arrival_check'      # SUCCEEDED인데 위치가 목표 범위 밖 (가짜 성공을 잡는 주 확인)
TF_STALE = 'tf_stale'                # 상태머신이 받은 map→odom이 오래됨 (보조 확인)
TIMEOUT = 'timeout'                  # 상태머신 쪽 제한 시간 초과
ABORTED = 'aborted'                  # Nav2가 ABORTED 반환
REJECTED = 'rejected'                # 목표 거부
CANCELED = 'canceled'                # 상태머신이 요청하지 않은 취소
CLOCK_STOPPED = 'clock_stopped'      # /clock 정지 (wall time 감시)

# NAV_NOT_READY의 detail: 마지막으로 확인된 준비 안 된 이유
LIFECYCLE_INACTIVE = 'lifecycle_inactive'
NAV_SERVER_UNAVAILABLE = 'nav_server_unavailable'
NO_READY_STATUS = 'no_ready_status'
