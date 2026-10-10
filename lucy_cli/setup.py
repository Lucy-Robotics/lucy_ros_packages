from setuptools import find_packages
from setuptools import setup

package_name = 'lucy_cli'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mael-RABOT',
    maintainer_email='contact@lucy-robotics.com',
    description='CLI package for Lucy',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'tui = lucy_cli.tui_node:main'
        ],
    },
)