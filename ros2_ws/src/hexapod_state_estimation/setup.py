from setuptools import find_packages, setup

package_name = "hexapod_state_estimation"

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
    description="IMU attitude estimate (/state/attitude); TerrainFeatures fusion not implemented yet.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "state_estimator_node = hexapod_state_estimation.state_estimator_node:main",
        ],
    },
)
