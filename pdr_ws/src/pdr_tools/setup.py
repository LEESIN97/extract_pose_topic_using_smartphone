from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pdr_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leeshin',
    maintainer_email='gkdlgus97@naver.com',
    description='Pose→Odometry & Path; TF broadcaster',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # lib/pdr_tools/pose_to_odom_path 로 설치됨
            'pose_to_odom_path = pdr_tools.pose_to_odom_path:main',
        ],
    },
)

