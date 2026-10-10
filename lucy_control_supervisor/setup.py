from setuptools import find_packages
from setuptools import setup

package_name = 'lucy_control_supervisor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/control_supervisor.launch.py']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Lucy Robotics Team',
    maintainer_email='contact@lucy-robotics.com',
    description='Restart RSP + ros2_control_node + spawners after config pipeline reload.',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'control_supervisor_node = lucy_control_supervisor.supervisor_node:main',
        ],
    },
)
