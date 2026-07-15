# Last updated: 2026-07-15
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "signal_tracker_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jaeseong",
    maintainer_email="jaeseong@example.com",
    description=(
        "Fixed-base upper-body tracker robot (torso + 2 arms + fingers) driven by "
        "Signal-Vision's /upper_body_pose topic."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motion_retarget_node = signal_tracker_robot.motion_retarget_node:main",
        ],
    },
)
