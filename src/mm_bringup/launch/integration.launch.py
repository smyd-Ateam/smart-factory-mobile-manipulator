"""Integrated mobile-manipulator bringup skeleton.

Each component remains disabled until its prerequisite PR is merged.
"""

from launch import LaunchDescription


def generate_launch_description():
    """Return the future integrated bringup launch description."""
    launch_description = LaunchDescription()

    # TODO(PR #4 merge): mm_description -- Gazebo and chassis.
    # from launch.actions import IncludeLaunchDescription
    # from launch.launch_description_sources import PythonLaunchDescriptionSource
    # from ament_index_python.packages import get_package_share_directory
    # launch_description.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([
    #             get_package_share_directory("mm_description"),
    #             "/launch/sim.launch.py",
    #         ])
    #     )
    # )

    # TODO(PR #4 merge): mm_navigation -- Nav2 (standalone:=false).
    # launch_description.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([
    #             get_package_share_directory("mm_navigation"),
    #             "/launch/nav2.launch.py",
    #         ]),
    #         launch_arguments={"standalone": "false"}.items(),
    #     )
    # )

    # TODO(feat/yolo PR merge): mm_perception -- YOLO detector.
    # launch_description.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([
    #             get_package_share_directory("mm_perception"),
    #             "/launch/detector.launch.py",
    #         ])
    #     )
    # )

    # TODO(feat/visual-servoing PR merge): visual_servoing -- align_node.
    # from launch_ros.actions import Node
    # launch_description.add_action(
    #     Node(package="visual_servoing", executable="align_node", output="screen")
    # )

    # TODO(feat/state-machine PR creation and merge): mm_task_manager -- state machine.
    # launch_description.add_action(
    #     Node(package="mm_task_manager", executable="task_manager", output="screen")
    # )

    # TODO(feat/grasp PR merge and observe_pose PR #5 confirmation): mm_grasp -- grasping.
    # launch_description.add_action(
    #     Node(package="mm_grasp", executable="grasp_node", output="screen")
    # )

    return launch_description
