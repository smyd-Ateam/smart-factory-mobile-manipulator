from glob import glob

from setuptools import find_packages, setup

package_name = 'mm_task_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/mock', glob('config/mock/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='A',
    maintainer_email='todo@todo.com',
    description='Mobile manipulator task state machine',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager = mm_task_manager.task_manager_node:main',
            # 시험용 mock
            'mock_clock = mm_task_manager.mocks.mock_clock:main',
            'mock_lifecycle = mm_task_manager.mocks.mock_lifecycle:main',
            'mock_tf = mm_task_manager.mocks.mock_tf:main',
            'mock_nav_server = mm_task_manager.mocks.mock_nav_server:main',
            'cmd_recorder = mm_task_manager.mocks.cmd_recorder:main',
            'mock_cmd_publisher = mm_task_manager.mocks.mock_cmd_publisher:main',
            'mock_align_server = mm_task_manager.mocks.mock_align_server:main',
            'mock_grasp_server = mm_task_manager.mocks.mock_grasp_server:main',
            'mock_target_pose = mm_task_manager.mocks.mock_target_pose:main',
            # 제안 초안 (A), B 확인 전
            'mock_observe_pose_server = mm_task_manager.mocks.mock_observe_pose_server:main',
        ],
    },
)
