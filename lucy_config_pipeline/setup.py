from setuptools import find_packages
from setuptools import setup

package_name = 'lucy_config_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/config_pipeline.launch.py']),
    ],
    install_requires=['setuptools', 'PyYAML', 'Jinja2'],
    extras_require={
        'test': [
            'pytest',
        ],
    },
    zip_safe=True,
    maintainer='Sentience Robotics Team',
    maintainer_email='contact@sentience-robotics.fr',
    description='Config store services + pipeline action server for Lucy hardware YAML.',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'config_pipeline_node = src.main:main',
            'client_registry_node = src.services.client_registry_node:main',
        ],
    },
)
