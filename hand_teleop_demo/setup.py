from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'hand_teleop_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/hand_teleop_demo']),
        ('share/hand_teleop_demo', ['package.xml']),
        ('share/hand_teleop_demo/launch', glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chengwsam',
    maintainer_email='lichengwang0727@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
