from setuptools import find_packages
from setuptools import setup

package_name = 'lucy_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucy Robotics Team',
    maintainer_email='contact@lucy-robotics.com',
    description='End-effector position and inverse kinematics with Pinocchio.',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'kinematics_node = lucy_kinematics.kinematics_node:main',
        ],
    },
)
