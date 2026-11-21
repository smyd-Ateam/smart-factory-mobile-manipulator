import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = get_package_share_directory('mm_description')
    xacro = os.path.join(pkg, 'urdf', 'mm_robot.urdf.xacro')
    world = PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')])
    render = LaunchConfiguration('render_engine')
    software_gl = LaunchConfiguration('software_gl')
    headless = LaunchConfiguration('headless')
    gz_args = PythonExpression(["'", world, " -r -v 3 --render-engine ", render,
                                "' + (' -s' if '", headless, "' == 'true' else '')"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items())
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro, ' use_vj:=false fix_gripper:=true']), value_type=str), 'use_sim_time': True}], output='screen')
    spawn = Node(package='ros_gz_sim', executable='create', arguments=['-topic', 'robot_description', '-name', 'mm_robot', '-z', '0.01'], output='screen')
    rgb = Node(package='ros_gz_bridge', executable='parameter_bridge', name='rgb_parameter_bridge',
        arguments=['/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image'],
        remappings=[('/rgbd_camera/image', '/camera/image_raw')], output='screen')
    depth = Node(package='ros_gz_bridge', executable='parameter_bridge', name='depth_parameter_bridge',
        arguments=['/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image'],
        remappings=[('/rgbd_camera/depth_image', '/camera/depth/image_raw')], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='yolo_test_room.sdf'),
        DeclareLaunchArgument('render_engine', default_value='ogre2'),
        DeclareLaunchArgument('software_gl', default_value='true'),
        DeclareLaunchArgument('headless', default_value='true'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1', condition=IfCondition(software_gl)),
        gazebo, rsp, spawn, rgb, depth])
