from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'handeye_realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='szhuang',
    maintainer_email='szhuang@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot = handeye_realsense.robot_state_estimation:main',
            'handeye = handeye_realsense.handeye_estimation:main',
            'aruco = handeye_realsense.aruco_estimation:main',
            'eye2hand = handeye_realsense.publish_eye2hand:main',
            'joint_positions = handeye_realsense.joint_positions:main',
        ],
    },
)
