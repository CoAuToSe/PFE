from setuptools import find_packages, setup

package_name = 'tello_path_follow'

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
    maintainer='thomas',
    maintainer_email='thomas.vita@cpe.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'path_follower = tello_path_follow.path_follower:main'
        'obstacle_detector = tello_path_follow.obstacle_detector:main'
        ],
    },
)
