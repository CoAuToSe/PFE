from setuptools import setup
from glob import glob
import os

package_name = 'drone_energy_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chunyu',
    maintainer_email='chunyu.zhang@ensta-paris.fr',
    description='Drone energy estimation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_state = drone_energy_estimation.battery_state:main',
        ],
    },
)