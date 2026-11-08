from setuptools import find_packages, setup

package_name = 'mm_grasp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunny',
    maintainer_email='jenny2203@ewhain.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'kinematics_solver = mm_grasp.kinematics_solver:main',
            'fk_validator = mm_grasp.fk_validator:main',
            'motion_planner = mm_grasp.motion_planner:main',
        ],
    },
)
