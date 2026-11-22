import os

import pytest

from mm_task_manager.mission import mission_from_dict
from mm_task_manager.pose_transition import NEAR_TO_PREGRASP_THRESHOLD_M

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'mission_test_room.yaml')
GOAL_MODE_CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'mock',
                                'mission_goal_mode.yaml')


def base_params(**overrides):
    params = {'goal.x': 1.0, 'goal.y': 2.0, 'goal.yaw': 0.5, 'target_class': 'block'}
    params.update(overrides)
    return params


def test_defaults():
    m, warnings = mission_from_dict(base_params())
    assert (m.goal_x, m.goal_y, m.goal_yaw, m.target_class) == (1.0, 2.0, 0.5, 'block')
    assert m.nav_only is True
    assert m.arrival_xy_tol == 0.15
    assert m.arrival_yaw_tol == 0.35
    assert m.cmd_rate_hz == 10.0
    assert m.cmd_input_timeout_sec == 0.5
    assert m.max_realign == 2
    assert m.standoff_list == [0.32, 0.32, 0.35]   # 하한 0.32 이상 placeholder (5.2, 조합 TBD)
    assert m.align_sm_timeout_code == 'ALIGN_TIMEOUT'
    assert m.grasp_timeout_code == 'UNKNOWN'
    assert m.ready_min_wait_sec == 1.0
    assert m.final_zero_sec == 1.0
    assert m.target_pose_max_x_m == 2.0      # C 제안 1차값 (PR #1 2026-11-20)
    assert m.target_pose_max_jump_m == 0.1
    assert len(warnings) == 1 and 'standoff_list' in warnings[0]   # placeholder [0.32, 0.32, 0.35] 중복 경고 (의도)


@pytest.mark.parametrize('key', ['ready_min_wait_sec', 'final_zero_sec'])
def test_non_negative_values(key):
    with pytest.raises(ValueError, match=key):
        mission_from_dict(base_params(**{key: -0.1}))
    m, _ = mission_from_dict(base_params(**{key: 0.0}))
    assert getattr(m, key) == 0.0


@pytest.mark.parametrize('missing', ['goal.x', 'goal.y', 'goal.yaw', 'target_class'])
def test_required_fields(missing):
    params = base_params()
    del params[missing]
    with pytest.raises(ValueError, match=missing):
        mission_from_dict(params)


@pytest.mark.parametrize('key', ['goal.x', 'goal.y', 'goal.yaw'])
@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_goal_must_be_finite(key, value):
    # 노드는 goal.*의 기본값을 NaN으로 선언한다 (YAML에 빠지면 여기서 걸림)
    with pytest.raises(ValueError, match=key):
        mission_from_dict(base_params(**{key: value}))


def test_empty_target_class_rejected():
    with pytest.raises(ValueError, match='target_class'):
        mission_from_dict(base_params(target_class=''))


def test_standoff_list_length_must_match_max_realign():
    with pytest.raises(ValueError, match='standoff_list'):
        mission_from_dict(base_params(standoff_list=[0.25, 0.22]))
    m, _ = mission_from_dict(base_params(max_realign=1, standoff_list=[0.25, 0.22]))
    assert m.standoff_list == [0.25, 0.22]


def test_standoff_must_be_positive():
    with pytest.raises(ValueError, match='standoff_list'):
        mission_from_dict(base_params(standoff_list=[0.25, 0.0, 0.28]))


def test_repeated_standoff_warns():
    # 5.2: 같은 standoff로 재정렬하면 같은 위치로 돌아와 반복 실패할 수 있음
    _, warnings = mission_from_dict(base_params(standoff_list=[0.25, 0.25, 0.28]))
    assert any('standoff_list' in w for w in warnings)


@pytest.mark.parametrize('key', [
    'ready_timeout_sec', 'tf_max_age_sec', 'nav_timeout_sec', 'cancel_timeout_sec',
    'arrival_xy_tol', 'arrival_yaw_tol', 'cmd_rate_hz', 'cmd_input_timeout_sec',
    'clock_watchdog_sec', 'align_timeout_sec', 'grasp_timeout_sec', 'target_pose_max_age_sec',
    'pregrasp_switch_distance_m', 'target_pose_max_x_m', 'target_pose_max_jump_m',
])
def test_positive_values(key):
    with pytest.raises(ValueError, match=key):
        mission_from_dict(base_params(**{key: 0.0}))


# ---- NaN · inf 검증 (> 0만 보면 NaN · inf가 통과: now > NaN이 항상 False라 시간 제한이 안 걸림) ----

POSITIVE_KEYS = [
    'ready_timeout_sec', 'nav_timeout_sec', 'align_timeout_sec', 'grasp_timeout_sec',
    'cancel_timeout_sec', 'clock_watchdog_sec', 'target_pose_max_age_sec', 'arrival_xy_tol',
    'arrival_yaw_tol', 'cmd_rate_hz', 'cmd_input_timeout_sec', 'tf_max_age_sec',
    'pregrasp_switch_distance_m', 'target_pose_max_x_m', 'target_pose_max_jump_m',
]


@pytest.mark.parametrize('value', [float('nan'), float('inf'), float('-inf')])
@pytest.mark.parametrize('key', POSITIVE_KEYS)
def test_positive_values_must_be_finite(key, value):
    with pytest.raises(ValueError, match=key):
        mission_from_dict(base_params(**{key: value}))


@pytest.mark.parametrize('value', [1e-9, 1e6])
@pytest.mark.parametrize('key', POSITIVE_KEYS)
def test_small_and_large_finite_positive_values_accepted(key, value):
    # 회귀: 유한한 양수는 경계(아주 작은 값 · 큰 값)도 그대로 통과
    m, _ = mission_from_dict(base_params(**{key: value}))
    assert getattr(m, key) == value


NON_NEGATIVE_KEYS = ['settle_sec', 'align_margin_sec', 'ready_min_wait_sec', 'final_zero_sec',
                     'nav_standoff_margin_m']


@pytest.mark.parametrize('value', [float('nan'), float('inf'), float('-inf')])
@pytest.mark.parametrize('key', NON_NEGATIVE_KEYS)
def test_non_negative_values_must_be_finite(key, value):
    # >= 0만 보면 NaN이 통과한다 (NaN < 0이 False)
    with pytest.raises(ValueError, match=key):
        mission_from_dict(base_params(**{key: value}))


@pytest.mark.parametrize('value', [0.0, 1e6])
@pytest.mark.parametrize('key', NON_NEGATIVE_KEYS)
def test_zero_and_large_finite_non_negative_values_accepted(key, value):
    # 회귀: 0(경계)과 큰 유한값은 그대로 통과
    m, _ = mission_from_dict(base_params(**{key: value}))
    assert getattr(m, key) == value


def test_negative_max_realign_rejected():
    with pytest.raises(ValueError, match='max_realign'):
        mission_from_dict(base_params(max_realign=-1, standoff_list=[]))


@pytest.mark.parametrize('key', ['align_sm_timeout_code', 'grasp_timeout_code'])
def test_timeout_code_must_be_known(key):
    with pytest.raises(ValueError, match=key):
        mission_from_dict(base_params(**{key: 'TIMEOUT'}))


def test_unknown_key_rejected():
    # 오타 난 파라미터가 조용히 무시되지 않게
    with pytest.raises(ValueError, match='nav_timeout'):
        mission_from_dict(base_params(nav_timeout=10.0))


def test_int_values_become_float():
    m, _ = mission_from_dict(base_params(**{'goal.x': 1, 'nav_timeout_sec': 90}))
    assert isinstance(m.goal_x, float)
    assert isinstance(m.nav_timeout_sec, float)


def _load_config(path):
    yaml = pytest.importorskip('yaml')
    with open(path) as f:
        params = yaml.safe_load(f)['task_manager']['ros__parameters']
    params.pop('use_sim_time', None)
    return params


def test_config_file_is_valid():
    m, warnings = mission_from_dict(_load_config(CONFIG))
    assert m.standoff_list == [0.32, 0.32, 0.35]
    assert m.nav_goal_mode == 'object'          # 물체 위치로 이동 목표 계산 (5.5)
    assert (m.object_x, m.object_y) == (1.40, 0.0)     # 목표 (0.93, 0, 0)
    assert len(warnings) == 1 and 'standoff_list' in warnings[0]   # placeholder [0.32, 0.32, 0.35] 중복 경고 (의도)


def test_goal_mode_config_file_is_valid():
    # mock 시나리오(S12)용: 이동 목표를 직접 지정
    params = _load_config(CONFIG)
    params.pop('object.x')
    params.pop('object.y')
    params.update(_load_config(GOAL_MODE_CONFIG))
    m, _ = mission_from_dict(params)
    assert m.nav_goal_mode == 'goal'
    assert (m.goal_x, m.goal_y, m.goal_yaw) == (1.0, 0.0, 0.0)


# ---- 이동 목표: goal.*와 object.* 중 정확히 하나 ----

def object_params(**overrides):
    params = {'object.x': 2.0, 'object.y': 1.0, 'target_class': 'block'}
    params.update(overrides)
    return params


def test_goal_mode():
    m, _ = mission_from_dict(base_params())
    assert m.nav_goal_mode == 'goal'
    assert (m.object_x, m.object_y) == (None, None)


def test_object_mode():
    m, _ = mission_from_dict(object_params())
    assert m.nav_goal_mode == 'object'
    assert (m.object_x, m.object_y) == (2.0, 1.0)
    assert (m.goal_x, m.goal_y, m.goal_yaw) == (None, None, None)


def test_object_and_goal_together_rejected():
    with pytest.raises(ValueError, match='exactly one'):
        mission_from_dict(dict(base_params(), **{'object.x': 2.0, 'object.y': 1.0}))


def test_neither_goal_nor_object_rejected():
    with pytest.raises(ValueError, match='exactly one'):
        mission_from_dict({'target_class': 'block'})


@pytest.mark.parametrize('missing', ['object.x', 'object.y'])
def test_partial_object_rejected(missing):
    params = object_params()
    del params[missing]
    with pytest.raises(ValueError, match=missing):
        mission_from_dict(params)


def test_partial_goal_with_object_rejected():
    # goal.*가 하나라도 있으면 goal 모드로 보고, 섞이면 거부
    with pytest.raises(ValueError):
        mission_from_dict(object_params(**{'goal.yaw': 0.0}))


@pytest.mark.parametrize('key', ['object.x', 'object.y'])
@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_object_must_be_finite(key, value):
    with pytest.raises(ValueError, match=key):
        mission_from_dict(object_params(**{key: value}))


def test_nav_standoff_margin():
    m, _ = mission_from_dict(object_params())
    assert m.nav_standoff_margin_m == 0.15                       # PR #1 D 의견 (0.10 → 0.15)
    assert m.approach_distance == pytest.approx(0.32 + 0.15)    # standoff_list[0] + 0.15
    m, _ = mission_from_dict(object_params(nav_standoff_margin_m=0.20,
                                           standoff_list=[0.30, 0.25, 0.35]))
    assert m.approach_distance == pytest.approx(0.50)
    with pytest.raises(ValueError, match='nav_standoff_margin_m'):
        mission_from_dict(object_params(nav_standoff_margin_m=-0.01))


def test_pregrasp_switch_distance_defaults_to_provisional_constant():
    # 기본값은 pose_transition의 상수 한 곳에서 온다 (희선님 확정, PR #1 2026-11-17)
    m, _ = mission_from_dict(base_params())
    assert m.pregrasp_switch_distance_m == NEAR_TO_PREGRASP_THRESHOLD_M


def test_pregrasp_switch_distance_can_be_overridden():
    m, _ = mission_from_dict(base_params(pregrasp_switch_distance_m=0.4))
    assert m.pregrasp_switch_distance_m == 0.4


# ---- approach_yaw (object 모드 접근 방향, interfaces.md 5.5 2026-11-23 확정) ----

def test_approach_yaw_default_zero():
    m, _ = mission_from_dict(object_params())
    assert m.approach_yaw == 0.0


def test_approach_yaw_in_test_room_yaml_is_zero():
    params = _load_config(CONFIG)
    assert params['approach_yaw'] == 0.0
    assert isinstance(params['approach_yaw'], float)   # ROS double 파라미터 (0이면 int로 읽혀 타입 오류)


def test_approach_yaw_int_converted_to_float():
    m, _ = mission_from_dict(object_params(approach_yaw=1))
    assert m.approach_yaw == 1.0 and isinstance(m.approach_yaw, float)


@pytest.mark.parametrize('value', [-10.0, -3.5, 3.5, 10.0])
def test_approach_yaw_no_range_limit(value):
    # 범위 정리는 geometry.wrap_angle이 맡음
    m, _ = mission_from_dict(object_params(approach_yaw=value))
    assert m.approach_yaw == value


@pytest.mark.parametrize('value', [float('nan'), float('inf'), float('-inf')])
def test_approach_yaw_must_be_finite(value):
    with pytest.raises(ValueError, match='approach_yaw'):
        mission_from_dict(object_params(approach_yaw=value))
