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
        # 노드 실행 파일은 2단계에서 추가
        'console_scripts': [],
    },
)
