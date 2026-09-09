from setuptools import find_packages
from setuptools import setup

package_name = 'lucy_modbus_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/modbus_bridge.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Sentience Robotics Team',
    maintainer_email='contact@sentience-robotics.fr',
    description='SHM → Modbus RTU bridge for Lucy RP2040 firmware',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'modbus_bridge_node = lucy_modbus_bridge.bridge_node:main',
        ],
    },
)
