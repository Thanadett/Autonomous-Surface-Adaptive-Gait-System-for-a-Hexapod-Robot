from setuptools import find_packages, setup

package_name = "hexapod_perception"

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
    description="Depth-camera terrain estimation. P2 -- not yet implemented.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "terrain_estimator_node = hexapod_perception.terrain_estimator_node:main",
            "voxel_map_node = hexapod_perception.voxel_map_node:main",
        ],
    },
)
