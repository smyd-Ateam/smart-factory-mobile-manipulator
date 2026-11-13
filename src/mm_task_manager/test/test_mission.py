import os

import pytest

from mm_task_manager.mission import mission_from_dict

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
    assert m.standoff_list == [0.25, 0.22, 0.28]
    assert m.align_sm_timeout_code == 'ALIGN_TIMEOUT'
    assert m.grasp_timeout_code == 'UNKNOWN'
    assert m.ready_min_wait_sec == 1.0
    assert m.final_zero_sec == 1.0
    assert warnings == []


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
])
def test_positive_values(key):
    with pytest.raises(ValueError, match=key):
        mission_from_dict(base_params(**{key: 0.0}))


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
    assert m.standoff_list == [0.25, 0.22, 0.28]
    assert m.nav_goal_mode == 'object'          # 물체 위치로 이동 목표 계산 (5.5)
    assert (m.object_x, m.object_y) == (1.40, 0.0)     # 스폰 (0, 0)에서 목표 (1.0, 0, 0)
    assert warnings == []


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
    assert m.approach_distance == pytest.approx(0.25 + 0.15)    # standoff_list[0] + 0.15
    m, _ = mission_from_dict(object_params(nav_standoff_margin_m=0.20,
                                           standoff_list=[0.30, 0.25, 0.35]))
    assert m.approach_distance == pytest.approx(0.50)
    with pytest.raises(ValueError, match='nav_standoff_margin_m'):
        mission_from_dict(object_params(nav_standoff_margin_m=-0.01))
