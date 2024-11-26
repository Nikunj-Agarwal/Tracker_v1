from setuptools import setup
import os
from glob import glob

package_name = 'servo_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='your.email@example.com',
    description='Servo controller package for ROS 2',
    license='Your License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_controller_node = servo_control.servo_controller:main',  # Node entry point
        ],
    },
)
