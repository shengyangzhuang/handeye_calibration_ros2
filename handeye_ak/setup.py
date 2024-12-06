from setuptools import find_packages, setup

package_name = 'handeye_ak'

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
    maintainer='szhuang',
    maintainer_email='heyeason@outlook.com',
    description='ROS2 package for hand-eye calibration for Azure Kinect depth camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot = handeye_ak.robot_state:main',
            'handeye = handeye_ak.calibrate_handeye:main',
            'aruco = handeye_ak.aruco:main',
            'eye2hand = handeye_ak.publish_eye2hand:main',
        ],
    },
)
