from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'final_code'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/final_code/launch', ['launch/sort_world.launch.py']),
        ('share/final_code/worlds', ['launch/worlds/sort_world.sdf']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A ROS2 package for trajectory generation and quaternion interpolation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_from_csv = final_code_code.trajectory_from_csv:main',
            'gen3lite_pymoveit2 = final_code.gen3lite_pymoveit2:main',
            'real_arm = final_code.real_arm:main',
            'move_to = final_code.move_to:main',
            'test = final_code.test:main',
            'gripper_position = final_code.gripper_position:main',
            'go_to_position = final_code.go_to_position:main',
            'arm = final_code.arm:main',
            'car = final_code.car:main',
            'car_bonus = final_code.car_bonus:main',
            'arm_bonus = final_code.arm_bonus:main',
        ],
    },
)
