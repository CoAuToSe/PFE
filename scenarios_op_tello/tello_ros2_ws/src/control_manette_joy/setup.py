from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'control_manette_joy'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
        glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thomas.vita',
    maintainer_email='thomas.vita@cpe.fr',
    description='Package permettant de controler un drone Tello avec une manette de Xbox360',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = control_manette_joy.control:main',  # Adapter selon l'emplacement du fichier Python
            'tello_behavior = control_manette_joy.tello_behavior:main',
            'client_service = control_manette_joy.client_service:main',
        ],
},
)
