from setuptools import find_packages, setup

package_name = "hexapod_locomotion"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/gait.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="T",
    maintainer_email="n.thanadett@gmail.com",
    description=(
        "Continuous-phase tripod/ripple/wave gait planner and the ROS node "
        "that drives the joint trajectory controller from it."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "locomotion_node = hexapod_locomotion.locomotion_node:main",
            "teleop_keyboard = hexapod_locomotion.teleop_keyboard:main",
        ],
    },
)
