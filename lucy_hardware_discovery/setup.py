from setuptools import find_packages, setup

package_name = "lucy_hardware_discovery"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/hardware_discovery.launch.py"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Sentience Robotics Team",
    maintainer_email="contact@sentience-robotics.fr",
    description="Hardware discovery services for Lucy (serial, V4L2, RealSense).",
    license="GPL-3.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hardware_discovery_node = lucy_hardware_discovery.discovery_node:main",
        ],
    },
)
