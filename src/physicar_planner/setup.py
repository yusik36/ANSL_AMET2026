import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'physicar_planner'

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
    description='Camera corridor planner: image to speed and steering',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = physicar_planner.planner_node:main',
        ],
    },
)
