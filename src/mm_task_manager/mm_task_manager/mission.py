"""미션 파라미터 읽기 · 검증. 기본값은 모두 임시값 (README TODO 참고)."""

from dataclasses import dataclass, field
from typing import List

from mm_task_manager import codes


@dataclass
class Mission:
    goal_x: float
    goal_y: float
    goal_yaw: float
    target_class: str

    nav_only: bool = True

    ready_timeout_sec: float = 30.0
    tf_max_age_sec: float = 0.5           # TODO: 3단계 측정 후 조정

    nav_timeout_sec: float = 90.0
    cancel_timeout_sec: float = 5.0
    settle_sec: float = 0.5
    arrival_xy_tol: float = 0.15
    arrival_yaw_tol: float = 0.35

    cmd_rate_hz: float = 10.0
    cmd_input_timeout_sec: float = 0.5
    clock_watchdog_sec: float = 3.0

    align_timeout_sec: float = 30.0       # TODO(D)
    align_margin_sec: float = 5.0
    align_sm_timeout_code: str = codes.ALIGN_TIMEOUT   # TODO(팀)

    grasp_timeout_sec: float = 60.0       # TODO(B)
    grasp_timeout_code: str = codes.UNKNOWN            # TODO(B)
    target_pose_max_age_sec: float = 1.0  # TODO(C, B)

    max_realign: int = 2
    # 0.25는 B 제안 초기 시험값, 0.22 · 0.28은 TODO(B) 임시 자리값
    standoff_list: List[float] = field(default_factory=lambda: [0.25, 0.22, 0.28])

    observe_pose_required: bool = False   # TODO(B, C)

    result_log_path: str = '~/.ros/mm_task_manager/results.jsonl'


# 파라미터 이름 → Mission 필드 이름
_REQUIRED = {'goal.x': 'goal_x', 'goal.y': 'goal_y', 'goal.yaw': 'goal_yaw',
             'target_class': 'target_class'}

_FLOAT_FIELDS = ('goal_x', 'goal_y', 'goal_yaw', 'ready_timeout_sec', 'tf_max_age_sec',
                 'nav_timeout_sec', 'cancel_timeout_sec', 'settle_sec', 'arrival_xy_tol',
                 'arrival_yaw_tol', 'cmd_rate_hz', 'cmd_input_timeout_sec',
                 'clock_watchdog_sec', 'align_timeout_sec', 'align_margin_sec',
                 'grasp_timeout_sec', 'target_pose_max_age_sec')

_POSITIVE_FIELDS = ('ready_timeout_sec', 'tf_max_age_sec', 'nav_timeout_sec',
                    'cancel_timeout_sec', 'arrival_xy_tol', 'arrival_yaw_tol', 'cmd_rate_hz',
                    'cmd_input_timeout_sec', 'clock_watchdog_sec', 'align_timeout_sec',
                    'grasp_timeout_sec', 'target_pose_max_age_sec')

_NON_NEGATIVE_FIELDS = ('settle_sec', 'align_margin_sec')


def _param_names():
    names = set(_REQUIRED)
    names.update(f for f in Mission.__dataclass_fields__ if f not in _REQUIRED.values())
    return names


def mission_from_dict(params):
    """파라미터 dict → (Mission, 경고 목록). 잘못된 값이면 ValueError."""
    unknown = set(params) - _param_names()
    if unknown:
        raise ValueError(f'unknown parameters: {sorted(unknown)}')

    kwargs = {}
    for name, field_name in _REQUIRED.items():
        if name not in params:
            raise ValueError(f'missing required parameter: {name}')
        kwargs[field_name] = params[name]
    for name, value in params.items():
        if name not in _REQUIRED:
            kwargs[name] = value

    for f in _FLOAT_FIELDS:
        if f in kwargs:
            kwargs[f] = float(kwargs[f])
    if 'standoff_list' in kwargs:
        kwargs['standoff_list'] = [float(v) for v in kwargs['standoff_list']]
    if 'max_realign' in kwargs:
        kwargs['max_realign'] = int(kwargs['max_realign'])

    m = Mission(**kwargs)
    _validate(m)
    return m, _warnings(m)


def _validate(m):
    if not m.target_class:
        raise ValueError('target_class must not be empty')
    for f in _POSITIVE_FIELDS:
        if getattr(m, f) <= 0.0:
            raise ValueError(f'{f} must be > 0 (got {getattr(m, f)})')
    for f in _NON_NEGATIVE_FIELDS:
        if getattr(m, f) < 0.0:
            raise ValueError(f'{f} must be >= 0 (got {getattr(m, f)})')
    if m.max_realign < 0:
        raise ValueError(f'max_realign must be >= 0 (got {m.max_realign})')
    if len(m.standoff_list) != m.max_realign + 1:
        raise ValueError(f'standoff_list needs max_realign + 1 = {m.max_realign + 1} values '
                         f'(got {len(m.standoff_list)})')
    if any(v <= 0.0 for v in m.standoff_list):
        raise ValueError(f'standoff_list values must be > 0 (got {m.standoff_list})')
    for f in ('align_sm_timeout_code', 'grasp_timeout_code'):
        if getattr(m, f) not in codes.ALL_CODES:
            raise ValueError(f'{f} must be one of {sorted(codes.ALL_CODES)} '
                             f'(got {getattr(m, f)})')


def _warnings(m):
    warnings = []
    if len(set(m.standoff_list)) != len(m.standoff_list):
        warnings.append(f'standoff_list has repeated values {m.standoff_list}: '
                        're-align may return to the same spot (interfaces.md 5.2)')
    return warnings
