from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'koch_ros2_wrapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), # copy the config file into share folder (for access when run ros2launch file)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "koch_follower_control = koch_ros2_wrapper.koch_follower_control:main",
            "koch_leader_control =  koch_ros2_wrapper.koch_leader_control:main",
            "koch_calibration = koch_ros2_wrapper.koch_arm_calibration:main",
            "koch_leader_follower = koch_ros2_wrapper.koch_leader_follower_control:main",
            "koch_vr_control = koch_ros2_wrapper.vr_control:main", 
        ],
    },
)
