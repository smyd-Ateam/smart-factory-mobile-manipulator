"""차체 + 라이다 시뮬 실행.

사용 예
  기본        : ros2 launch mm_description sim.launch.py
  Mac UTM VM  : ros2 launch mm_description sim.launch.py software_gl:=true headless:=true rviz_gpu:=true
  WSL2        : ros2 launch mm_description sim.launch.py software_gl:=true
  RViz 끄기   : rviz:=false 추가
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD_NAME = 'test_room'   # worlds/test_room.sdf 안의 <world name>
ROBOT_NAME = 'mm_robot'


def generate_launch_description():
    pkg = get_package_share_directory('mm_description')
    xacro_file = os.path.join(pkg, 'urdf', 'mm_base.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', WORLD_NAME + '.sdf')
    rviz_file = os.path.join(pkg, 'rviz', 'base.rviz')

    render_engine = LaunchConfiguration('render_engine')
    software_gl = LaunchConfiguration('software_gl')
    headless = LaunchConfiguration('headless')

    args = [
        DeclareLaunchArgument('render_engine', default_value='ogre',
                              description='ogre 또는 ogre2 (Mac VM, WSL 모두 ogre 권장)'),
        DeclareLaunchArgument('software_gl', default_value='false',
                              description='true면 LIBGL_ALWAYS_SOFTWARE=1 (WSL용)'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true면 Gazebo 화면 없이 서버만 실행'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='true면 RViz도 같이 실행 (rviz/base.rviz 설정 사용)'),
        DeclareLaunchArgument('rviz_gpu', default_value='false',
                              description='true면 software_gl과 상관없이 RViz만 GPU로 그림 (Mac VM용). '
                                          'false면 RViz도 software_gl 설정을 따름 (WSL은 false)'),
    ]

    # WSL: CPU 렌더링 강제
    software_env = SetEnvironmentVariable(
        'LIBGL_ALWAYS_SOFTWARE', '1', condition=IfCondition(software_gl))

    # Gazebo 실행 (-r: 바로 재생, -s: 서버만)
    gz_args = PythonExpression([
        "'", world_file, " -r -v 3 --render-engine ", render_engine, "'",
        " + (' -s' if '", headless, "' == 'true' else '')",
    ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', ROBOT_NAME, '-z', '0.01'],
        output='screen',
    )

    # Gazebo ↔ ROS 토픽 연결
    #   [ : Gazebo → ROS,  ] : ROS → Gazebo
    joint_state_gz = f'/world/{WORLD_NAME}/model/{ROBOT_NAME}/joint_state'
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            joint_state_gz + '@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        remappings=[(joint_state_gz, '/joint_states')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    rviz_on = LaunchConfiguration('rviz')
    rviz_gpu = LaunchConfiguration('rviz_gpu')

    # RViz (rviz_gpu:=false) : software_gl 설정을 그대로 따름
    rviz_default = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(
            ["'", rviz_on, "' == 'true' and '", rviz_gpu, "' != 'true'"])),
        output='screen',
    )

    # RViz (rviz_gpu:=true) : CPU 렌더링 설정을 끄고 GPU로 그림 (Mac VM)
    rviz_gpu_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '0'},
        condition=IfCondition(PythonExpression(
            ["'", rviz_on, "' == 'true' and '", rviz_gpu, "' == 'true'"])),
        output='screen',
    )

    return LaunchDescription(args + [software_env, gazebo, rsp, spawn, bridge,
                                     rviz_default, rviz_gpu_node])
