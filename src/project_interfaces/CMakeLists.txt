cmake_minimum_required(VERSION 3.8)
project(project_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# 필드는 docs/interfaces.md 4.1 Align, 4.2 Grasp 기준
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Align.action"
  "action/Grasp.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
