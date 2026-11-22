# 통합 1단계: feat/grasp 5741526 + feat/yolo + feat/visual-servoing + feat/state-machine (+ PR #4 nav2_params)
# 시험 폴더(STAGE1_DIR)와 팀 워크스페이스(TEAM_WS)는 환경변수로 바꿀 수 있음
STAGE1_DIR=${STAGE1_DIR:-/tmp/sfmm-brass-v7}
TEAM_WS=${TEAM_WS:-$HOME/smart-factory-mobile-manipulator}
_sfmm_env_opts=$-
set +u
source /opt/ros/humble/setup.bash
source "$TEAM_WS/install/setup.bash"
source "$STAGE1_DIR/install/setup.bash"
case $_sfmm_env_opts in *u*) set -u;; esac
unset _sfmm_env_opts
export IGN_GAZEBO_RESOURCE_PATH="$STAGE1_DIR/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export PYTHONNOUSERSITE=1
