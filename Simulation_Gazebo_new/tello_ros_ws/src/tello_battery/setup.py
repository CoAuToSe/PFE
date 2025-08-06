from setuptools import setup

package_name = 'tello_battery'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thomas VITA',
    maintainer_email='thomas.vita@cpe.fr',
    description='Estimation de la consommation de batterie pour un drone Tello',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_estimator = tello_battery.battery_estimator:main',
            'drone_sync_node = tello_battery.drone_sync_node:main',
        ],
    },
)
