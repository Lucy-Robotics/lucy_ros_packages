from setuptools import find_packages
from setuptools import setup

package_name = "lucy_eye_leds"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lucy Robotics Team",
    maintainer_email="contact@lucy-robotics.com",
    description="Keep the last eye LED colours for clients that connect later.",
    license="GPL-3.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "eye_leds_node = lucy_eye_leds.eye_leds_node:main",
        ],
    },
)
