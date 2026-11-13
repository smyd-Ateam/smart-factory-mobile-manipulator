"""미션 파라미터 읽기 · 검증. 기본값은 모두 임시값 (README TODO 참고)."""

import math
from dataclasses import dataclass, field
from typing import List, Optional

from mm_task_manager import codes


@dataclass
class Mission:
    target_class: str

    # 이동 목표: goal.*(직접 지정, map 기준)와 object.*(물체 위치, map 기준) 중 정확히 하나.
    # object.*면 물체에서 approach_distance만큼 떨어져 물체를 바라보는 목표를 계산 (5.5)
    goal_x: Optional[float] = None
    goal_y: Optional[float] = None
    goal_yaw: Optional[float] = None
    object_x: Optional[float] = None
    object_y: Optional[float] = None
    # 이동 거리 = standoff_list[0] + 이 값 (5.5). 0.10 → 0.15: 도착 오차(최대 0.126 m)가 물체 쪽으로
    # 생겨도 차체 앞면과 물체 사이 약 6 cm 남도록 (PR #1 D 의견)
    nav_standoff_margin_m: float = 0.15

    nav_only: bool = True

    ready_timeout_sec: float = 30.0
    ready_min_wait_sec: float = 1.0       # DDS discovery 대기 (다른 /cmd_vel 발행자 확인)
    final_zero_sec: float = 1.0           # 끝난 뒤 /cmd_vel 0을 보내는 시간, 그 뒤 노드 종료
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

    @property
    def nav_goal_mode(self):
        return 'object' if self.object_x is not None else 'goal'

    @property
    def approach_distance(self):
        """object 모드에서 물체 ~ 이동 목표 거리 (base_link 원점 기준)."""
        return self.standoff_list[0] + self.nav_standoff_margin_m


# 파라미터 이름 → Mission 필드 이름 (점이 들어간 이름만)
_GOAL_PARAMS = {'goal.x': 'goal_x', 'goal.y': 'goal_y', 'goal.yaw': 'goal_yaw'}
_OBJECT_PARAMS = {'object.x': 'object_x', 'object.y': 'object_y'}
_DOTTED = {**_GOAL_PARAMS, **_OBJECT_PARAMS}

_FLOAT_FIELDS = ('goal_x', 'goal_y', 'goal_yaw', 'object_x', 'object_y', 'nav_standoff_margin_m',
                 'ready_timeout_sec', 'ready_min_wait_sec',
                 'final_zero_sec', 'tf_max_age_sec',
                 'nav_timeout_sec', 'cancel_timeout_sec', 'settle_sec', 'arrival_xy_tol',
                 'arrival_yaw_tol', 'cmd_rate_hz', 'cmd_input_timeout_sec',
                 'clock_watchdog_sec', 'align_timeout_sec', 'align_margin_sec',
                 'grasp_timeout_sec', 'target_pose_max_age_sec')

_POSITIVE_FIELDS = ('ready_timeout_sec', 'tf_max_age_sec', 'nav_timeout_sec',
                    'cancel_timeout_sec', 'arrival_xy_tol', 'arrival_yaw_tol', 'cmd_rate_hz',
                    'cmd_input_timeout_sec', 'clock_watchdog_sec', 'align_timeout_sec',
                    'grasp_timeout_sec', 'target_pose_max_age_sec')

_NON_NEGATIVE_FIELDS = ('settle_sec', 'align_margin_sec', 'ready_min_wait_sec', 'final_zero_sec',
                        'nav_standoff_margin_m')


def _param_names():
    names = set(_DOTTED)
    names.update(f for f in Mission.__dataclass_fields__ if f not in _DOTTED.values())
    return names


def _nav_target(params):
    """goal.*와 object.* 중 정확히 하나가 빠짐없이 있는지 확인하고 해당 dict를 돌려준다."""
    has_goal = any(k in params for k in _GOAL_PARAMS)
    has_object = any(k in params for k in _OBJECT_PARAMS)
    if has_goal == has_object:
        raise ValueError('set exactly one of goal.{x,y,yaw} (nav goal) or object.{x,y} '
                         f'(object position) (got goal: {has_goal}, object: {has_object})')
    group = _GOAL_PARAMS if has_goal else _OBJECT_PARAMS
    missing = [k for k in group if k not in params]
    if missing:
        raise ValueError(f'missing required parameter: {", ".join(missing)}')
    return group


def mission_from_dict(params):
    """파라미터 dict → (Mission, 경고 목록). 잘못된 값이면 ValueError."""
    unknown = set(params) - _param_names()
    if unknown:
        raise ValueError(f'unknown parameters: {sorted(unknown)}')

    if 'target_class' not in params:
        raise ValueError('missing required parameter: target_class')
    group = _nav_target(params)

    kwargs = {}
    for name, value in params.items():
        kwargs[_DOTTED.get(name, name)] = value

    for f in _FLOAT_FIELDS:
        if f in kwargs:
            kwargs[f] = float(kwargs[f])
    if 'standoff_list' in kwargs:
        kwargs['standoff_list'] = [float(v) for v in kwargs['standoff_list']]
    if 'max_realign' in kwargs:
        kwargs['max_realign'] = int(kwargs['max_realign'])

    m = Mission(**kwargs)
    _validate(m, group)
    return m, _warnings(m)


def _validate(m, group):
    for name, field_name in group.items():
        if not math.isfinite(getattr(m, field_name)):
            raise ValueError(f'{name} must be a finite number (got {getattr(m, field_name)})')
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
