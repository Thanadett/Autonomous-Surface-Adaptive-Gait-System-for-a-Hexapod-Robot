from setuptools import find_packages, setup

package_name = "hexapod_gait_selector"

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
        "AUTO gait selection: rule-based baseline + safety filter now, "
        "ONNX-based learned model (P4) to follow."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gait_selector_node = hexapod_gait_selector.gait_selector_node:main",
        ],
    },
)
