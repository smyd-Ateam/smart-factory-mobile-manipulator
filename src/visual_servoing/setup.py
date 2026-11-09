from glob import glob

from setuptools import setup

package_name = 'visual_servoing'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yunkong03',
    maintainer_email='yundaeun@g.hongik.ac.kr',
    description='Visual servoing: 카메라 피드백으로 모바일 베이스 미세 정렬 (D)',
    license='TODO',
    entry_points={
        'console_scripts': [
            'align_node = visual_servoing.align_node:main',
            'mock_scene_node = visual_servoing.mock_scene_node:main',
        ],
    },
)
