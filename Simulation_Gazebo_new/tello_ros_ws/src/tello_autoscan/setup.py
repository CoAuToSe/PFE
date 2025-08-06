from setuptools import setup

package_name = 'tello_autoscan'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thomas V',
    maintainer_email='thomas.vita@cpe.fr',
    description='Scan automatique du bâtiment avec Tello',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tello_scan = tello_autoscan.tello_scan:main',
        ],
    },
)
