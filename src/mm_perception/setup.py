from glob import glob
from setuptools import setup

package_name = "mm_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="C (perception)",
    description="YOLO part detector node",
    license="TODO",
    entry_points={"console_scripts": ["yolo_detector = mm_perception.yolo_detector:main"]},
)
