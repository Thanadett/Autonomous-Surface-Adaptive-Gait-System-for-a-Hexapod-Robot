from glob import glob

from setuptools import find_packages, setup

package_name = "hexapod_evaluation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="T",
    maintainer_email="n.thanadett@gmail.com",
    description="Experiment runners and metrics for the simulation test plan (E1-E7).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "e1_ik_offline = hexapod_evaluation.e1_ik_offline:main",
            "e1_sim = hexapod_evaluation.e1_sim_node:main",
            "e2_runner = hexapod_evaluation.e2_runner:main",
            "e2_report = hexapod_evaluation.e2_report:main",
            "e2_reevaluate = hexapod_evaluation.e2_reevaluate:main",
        ],
    },
)
