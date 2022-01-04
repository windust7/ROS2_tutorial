#!/usr/bin/env python3

import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'uniform_circle_turtle'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        (share_dir + '/param', glob.glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='windust7',
    maintainer_email='dkcharng@naver.com',
    description='ROS2 example for turtlessim(unifrom circular motion)',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rvd_publisher = uniform_circle_turtle.rvd_publisher:main',
            'rvd_subscriber = uniform_circle_turtle.rvd_subscriber:main'
        ],
    },
)
