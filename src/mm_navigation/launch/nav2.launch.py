"""시뮬 + 지도 + AMCL + Nav2 + RViz.

사용 예
  Mac UTM VM : ros2 launch mm_navigation nav2.launch.py software_gl:=true headless:=true rviz_gpu:=true standalone:=true
  WSL2       : ros2 launch mm_navigation nav2.launch.py software_gl:=true standalone:=true

시뮬 옵션(software_gl, headless, rviz, rviz_gpu, render_engine)은 sim.launch.py로 그대로 전달된다.

속도 흐름 (docs/interfaces.md 3.1)
  controller_server -> cmd_vel_nav -> velocity_smoother -> /nav2/cmd_vel -> 상태머신 -> /cmd_vel
  behavior_server(복구 동작)                            -> /nav2/cmd_vel

standalone:=true면 상태머신 없이 테스트하도록 /nav2/cmd_vel 대신 /cmd_vel로 바로 보낸다.
standalone:=false(기본)에서는 상태머신이 없으면 차체가 움직이지 않는다.

기본 navigation_launch.py는 velocity_smoother와 behavior_server의 출력을 바꿀 수 없어서
같은 노드들을 여기서 직접 띄운다. 위치 추정은 바꿀 게 없어서 기본 launch를 그대로 쓴다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
]


def generate_launch_description():
    pkg = get_package_share_directory('mm_navigation')
    bringup = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    standalone = LaunchConfiguration('standalone')

    args = [
        DeclareLaunchArgument('map',
                              default_value=os.path.join(pkg, 'maps', 'test_room.yaml'),
                              description='지도 yaml 경로'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
                              description='Nav2 파라미터 파일 경로'),
        DeclareLaunchArgument('standalone', default_value='false',
                              description='true면 상태머신 없이 Nav2 출력을 /cmd_vel로 바로 보냄 (테스트용)'),
    ]

    nav_out = PythonExpression(
        ["'/cmd_vel' if '", standalone, "' == 'true' else '/nav2/cmd_vel'"])

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mm_description'), 'launch', 'sim.launch.py')),
        launch_arguments={
            'rviz_config': os.path.join(pkg, 'rviz', 'nav.rviz'),
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    def nav_node(package, executable, remappings=None):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            parameters=[params_file],
            remappings=remappings or [],
        )

    nav_nodes = [
        nav_node('nav2_controller', 'controller_server',
                 [('cmd_vel', 'cmd_vel_nav')]),
        nav_node('nav2_smoother', 'smoother_server'),
        nav_node('nav2_planner', 'planner_server'),
        # 복구 동작(spin, backup)도 velocity_smoother를 거치지 않고 cmd_vel로 바로 낸다.
        # 상태머신을 건너뛰지 않도록 같은 출력으로 보낸다.
        nav_node('nav2_behaviors', 'behavior_server',
                 [('cmd_vel', nav_out)]),
        nav_node('nav2_bt_navigator', 'bt_navigator'),
        nav_node('nav2_waypoint_follower', 'waypoint_follower'),
        nav_node('nav2_velocity_smoother', 'velocity_smoother',
                 [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', nav_out)]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': LIFECYCLE_NODES,
            }],
        ),
    ]

    return LaunchDescription(args + [sim, localization] + nav_nodes)
