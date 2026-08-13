import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'physicar_imu'

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
    description='ROS2 driver for HandsFree-protocol USB IMU',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hfi_imu_node = physicar_imu.hfi_imu_node:main',
        ],
    },
)
