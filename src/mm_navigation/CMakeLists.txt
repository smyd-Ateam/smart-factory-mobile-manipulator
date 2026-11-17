cmake_minimum_required(VERSION 3.8)
project(mm_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # 시뮬 테스트: MM_SIM_TEST=1 일 때만 Gazebo + Nav2를 띄우고, 아니면 skip
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/sim/test_spin_at_goal.py TIMEOUT 900)
  # SLAM max_laser_range가 차체 기울기로 라이다가 바닥에 닿는 거리보다 짧은지 (xacro 치수로 계산)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_lidar_floor_range test/test_lidar_floor_range.py)
endif()

ament_package()
