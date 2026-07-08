from glob import glob
from setuptools import find_packages, setup

package_name = "linker_manipulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="robot_dev",
    maintainer_email="user@example.com",
    description="ROS2 nodes for Linker A7 and L20 Lite tactile manipulation demos.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "a7_driver_node = linker_manipulation.a7_driver_node:main",
            "l20lite_driver_node = linker_manipulation.l20lite_driver_node:main",
            "grasp_controller_node = linker_manipulation.grasp_controller_node:main",
            "demo_task_node = linker_manipulation.demo_task_node:main",
        ],
    },
)
