from setuptools import find_packages, setup

package_name = 'camera_input'

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
    maintainer='nikunj',
    maintainer_email='nikunj.agarwal.ug23@plaksha.edu.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_input.camera:main',
            'ball_detector = camera_input.ball_detection:main',
        ],
    },
)