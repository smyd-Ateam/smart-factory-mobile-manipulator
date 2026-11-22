"""Gazebo 카메라로 Align 액션 서버 시험 (시각 피드백 정렬 테스트).

Gazebo(화면 없이, ogre2) + 브리지 + 정답 검출 중계 + Align 서버를 띄운다.
실제로 렌더링된 깊이 영상과 boundingbox 카메라 박스로 정렬한다.

  ros2 launch visual_servoing sim_align.launch.py
  # 카메라 위치: test(임시, 기본) / near / pregrasp (sim_model.CAMERA_POSES)
  ros2 launch visual_servoing sim_align.launch.py camera_pose:=near
  # 같은 클래스 방해물(pin 하나 더, 목표 옆 0.18 m)을 넣어 목표를 계속 따라가는지 확인
  ros2 launch visual_servoing sim_align.launch.py camera_pose:=near distractor:=true
  # YOLO 검출 노드(C)를 따로 띄워 /camera/image_raw → /perception/detections 로 쓸 때
  ros2 launch visual_servoing sim_align.launch.py gt_detections:=false
  # 목표를 block으로 (feat/yolo block과 같은 모양·색·label 1)
  ros2 launch visual_servoing sim_align.launch.py target:=block
  # 검출이 느린 PC에서 검출 품질만 볼 때: 시뮬레이션을 느리게 (모든 노드가 시뮬레이션 시간 사용)
  ros2 launch visual_servoing sim_align.launch.py gt_detections:=false rtf:=0.1

반복 시험 (다른 터미널):
  ros2 run visual_servoing mock_trials --ros-args -p scene:=sim -p use_sim_time:=true \
    -p settle_sec:=2.0 -p wall_factor:=5.0 -p trials:=20
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from visual_servoing.geometry import matrix_to_quat, optical_rotation_in_base
from visual_servoing.sim_model import CAMERA_POSES, sdf_pose

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


PIN_GEOMETRY = '<cylinder><radius>0.01</radius><length>0.05</length></cylinder>'
# feat/yolo perception/synthetic_data/models/block: 2.5 × 2.5 × 4 cm, 파랑, label 1
BLOCK_SWAPS = [
    ('<pose>0 0 0.025 0 0 0</pose>', '<pose>0 0 0.02 0 0 0</pose>'),
    (PIN_GEOMETRY, '<box><size>0.025 0.025 0.04</size></box>'),
    ('<ambient>0.55 0.45 0.12 1</ambient>', '<ambient>0.15 0.35 0.75 1</ambient>'),
    ('<diffuse>0.78 0.65 0.20 1</diffuse>', '<diffuse>0.15 0.35 0.75 1</diffuse>'),
    ('<specular>0.95 0.85 0.45 1</specular>', '<specular>0.3 0.3 0.3 1</specular>'),
    ('<label>2</label>', '<label>1</label>'),
]


def as_block(target_model):
    """목표 pin 모델을 feat/yolo block 모양으로 바꾼다 (이름 pin_target은 그대로)."""
    for old, new in BLOCK_SWAPS:
        if old not in target_model:
            raise RuntimeError(f'목표 모델에 "{old}"가 없음')
        target_model = target_model.replace(old, new)
    return target_model


def distractor_model(target_model):
    """목표 pin 모델을 복사해 이름과 위치(목표 옆 y -0.18 m)만 바꾼다."""
    model = target_model.replace('<model name="pin_target">', '<model name="pin_distractor">')
    return model.replace('<pose>0.60 0.08 0 0 0 0</pose>', '<pose>0.60 -0.10 0 0 0 0</pose>')


def world_with_camera(template_path, pose, distractor=False, rtf=1.0, target='pin'):
    """월드 템플릿의 카메라 위치(test 값)·RTF를 바꾼 임시 SDF 경로."""
    with open(template_path, encoding='utf-8') as f:
        sdf = f.read()
    rtf_tag = '<real_time_factor>1.0</real_time_factor>'
    if sdf.count(rtf_tag) != 1:
        raise RuntimeError(f'{template_path}: "{rtf_tag}"가 1곳에 있어야 함')
    sdf = sdf.replace(rtf_tag, f'<real_time_factor>{rtf}</real_time_factor>')
    old = sdf_pose(CAMERA_POSES['test'])
    if sdf.count(old) != 2:
        raise RuntimeError(f'{template_path}: 카메라 pose "{old}"가 2곳(rgbd, bbox)에 있어야 함')
    sdf = sdf.replace(old, sdf_pose(pose))
    start = sdf.index('<model name="pin_target">')
    end = sdf.index('</model>', start) + len('</model>')
    if target == 'block':
        sdf = sdf[:start] + as_block(sdf[start:end]) + sdf[end:]
        end = sdf.index('</model>', start) + len('</model>')
    if distractor:
        sdf = sdf[:end] + '\n' + distractor_model(sdf[start:end]) + sdf[end:]
    out = tempfile.NamedTemporaryFile('w', suffix='.sdf', prefix='align_sim_', delete=False)
    with out:
        out.write(sdf)
    return out.name


def launch_setup(context):
    name = LaunchConfiguration('camera_pose').perform(context)
    if name not in CAMERA_POSES:
        raise RuntimeError(f'camera_pose는 {list(CAMERA_POSES)} 중 하나: {name!r}')
    pose = CAMERA_POSES[name]
    share = get_package_share_directory('visual_servoing')
    distractor = LaunchConfiguration('distractor').perform(context).lower() == 'true'
    rtf = float(LaunchConfiguration('rtf').perform(context))
    if not 0.0 < rtf <= 1.0:
        raise RuntimeError(f'rtf는 0 초과 1 이하: {rtf}')
    target = LaunchConfiguration('target').perform(context)
    if target not in ('pin', 'block'):
        raise RuntimeError(f'target은 pin 또는 block: {target!r}')
    world = world_with_camera(os.path.join(share, 'worlds', 'align_sim.sdf'), pose, distractor,
                              rtf, target)
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
    # 월드 카메라와 같은 위치로 base_link → camera_optical_frame
    qx, qy, qz, qw = matrix_to_quat(optical_rotation_in_base(pose.pitch))
    camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', output='log',
        arguments=['--x', str(pose.x), '--y', '0', '--z', str(pose.z),
                   '--qx', str(qx), '--qy', str(qy), '--qz', str(qz), '--qw', str(qw),
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_optical_frame'],
        parameters=[sim_time],
    )
    scene = Node(package='visual_servoing', executable='sim_scene_node', output='screen',
                 parameters=[sim_time, {'gt_detections': ParameterValue(
                     LaunchConfiguration('gt_detections'), value_type=bool)}])
    align = Node(package='visual_servoing', executable='align_node', output='screen',
                 parameters=[sim_time])
    return [gazebo, bridge, camera_tf, scene, align]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gt_detections', default_value='true',
            description='true = Gazebo 정답 박스로 검출, false = 외부 검출 노드(YOLO) 사용'),
        DeclareLaunchArgument(
            'camera_pose', default_value='test',
            description=f'카메라 위치 {list(CAMERA_POSES)}'),
        DeclareLaunchArgument(
            'distractor', default_value='false',
            description='true면 같은 클래스(pin) 방해물을 목표 옆에 하나 더 둔다'),
        DeclareLaunchArgument(
            'target', default_value='pin', description='목표 물체 pin 또는 block'),
        DeclareLaunchArgument(
            'rtf', default_value='1.0',
            description='Gazebo 실시간 배율 목표. 검출이 느린 PC에서 0.1 등으로 낮춘다'),
        OpaqueFunction(function=launch_setup),
    ])
