import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'physicar_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ansl207',
    maintainer_email='ansl207@example.com',
    description='Real-vehicle Ackermann (/speed + /steering) driver for the practice platform ESC+servo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_node = physicar_driver.driver_node:main',
        ],
    },
)
