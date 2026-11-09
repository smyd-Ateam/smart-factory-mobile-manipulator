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
        ],
    },
)
