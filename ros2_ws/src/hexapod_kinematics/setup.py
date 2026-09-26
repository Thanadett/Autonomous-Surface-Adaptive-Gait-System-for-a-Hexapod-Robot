from setuptools import find_packages, setup

package_name = "hexapod_kinematics"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="T",
    maintainer_email="n.thanadett@gmail.com",
    description=(
        "Forward/inverse kinematics for the six 3-DOF CAD leg chains, "
        "parsed directly from the expanded robot_description."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
)
