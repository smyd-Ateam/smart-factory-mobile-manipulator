"""Gazebo 카메라로 Align 액션 서버 시험 (시각 피드백 정렬 테스트).

Gazebo(화면 없이, ogre2) + 브리지 + 정답 검출 중계 + Align 서버를 띄운다.
실제로 렌더링된 깊이 영상과 boundingbox 카메라 박스로 정렬한다. YOLO는 아직 안 쓴다.

  ros2 launch visual_servoing sim_align.launch.py
  # YOLO 검출 노드(C)를 따로 띄워 /camera/image_raw → /perception/detections 로 쓸 때
  ros2 launch visual_servoing sim_align.launch.py gt_detections:=false

반복 시험 (다른 터미널):
  ros2 run visual_servoing mock_trials --ros-args -p scene:=sim -p use_sim_time:=true \
    -p settle_sec:=2.0 -p wall_factor:=5.0 -p trials:=20
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from visual_servoing.geometry import matrix_to_quat, optical_rotation_in_base
from visual_servoing.mock_model import CameraMount

MODEL = 'mm_test_base'

# Gazebo 토픽 @ ROS 타입 [ Gazebo 타입  ([ = Gazebo→ROS, ] = ROS→Gazebo)
BRIDGES = [
    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    '/sim_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
    '/sim_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
    '/sim_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
    '/sim_boxes@vision_msgs/msg/Detection2DArray[ignition.msgs.AnnotatedAxisAligned2DBox_V',
    f'/model/{MODEL}/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
    f'/model/{MODEL}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
]
REMAPS = [
    ('/sim_camera/image', '/camera/image_raw'),
    ('/sim_camera/depth_image', '/camera/depth/image_raw'),
    ('/sim_camera/camera_info', '/camera/camera_info'),
    ('/sim_boxes', '/servoing/sim_boxes'),
    (f'/model/{MODEL}/pose', '/servoing/sim_base_pose'),
    (f'/model/{MODEL}/cmd_vel', '/servoing/cmd_vel'),
]


def generate_launch_description():
    share = get_package_share_directory('visual_servoing')
    world = os.path.join(share, 'worlds', 'align_sim.sdf')
    sim_time = {'use_sim_time': True}

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', world],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        output='log',
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='log',
        arguments=BRIDGES, remappings=REMAPS, parameters=[sim_time],
    )

    # align_sim.sdf 카메라 위치와 같아야 한다 (test_sim.py가 확인)
    mount = CameraMount()
    qx, qy, qz, qw = matrix_to_quat(optical_rotation_in_base(mount.pitch))
    camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', output='log',
        arguments=['--x', str(mount.x), '--y', '0', '--z', str(mount.z),
                   '--qx', str(qx), '--qy', str(qy), '--qz', str(qz), '--qw', str(qw),
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_optical_frame'],
        parameters=[sim_time],
    )
    gt_arg = DeclareLaunchArgument(
        'gt_detections', default_value='true',
        description='true = Gazebo 정답 박스로 검출, false = 외부 검출 노드(YOLO) 사용')
    scene = Node(package='visual_servoing', executable='sim_scene_node', output='screen',
                 parameters=[sim_time, {'gt_detections': ParameterValue(
                     LaunchConfiguration('gt_detections'), value_type=bool)}])
    align = Node(package='visual_servoing', executable='align_node', output='screen',
                 parameters=[sim_time])
    return LaunchDescription([gt_arg, gazebo, bridge, camera_tf, scene, align])
