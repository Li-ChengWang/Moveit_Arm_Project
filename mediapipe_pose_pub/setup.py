from setuptools import find_packages, setup

package_name = 'mediapipe_pose_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mediapipe_moveit_follow.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml','config/moveit_py_wildcard.yaml',]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chengwsam',
    maintainer_email='lichengwang0727@gmail.com',
    description='MediaPipe + RealSense to publish PoseStamped for MoveIt following',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mediapipe_pose_publisher = mediapipe_pose_pub.pose_pub_node:main',
            'moveit_pose_follower = mediapipe_pose_pub.moveit_follower_node:main',
        ],
    },
)
