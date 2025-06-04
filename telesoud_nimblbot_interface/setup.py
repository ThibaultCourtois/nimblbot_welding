from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'telesoud_nimblbot_interface'

setup(
    name=package_name,
    version='0.9.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'welding_meshes'), glob('welding_meshes/*.stl')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thibault Courtois',
    maintainer_email='tcourtois@nimbl-bot.com',
    description='ROS2 package for Telesoud WeezTouch controller and NimblBot robots interface',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'translator = telesoud_nimblbot_interface.translator:main',
            'welding_command_handler = telesoud_nimblbot_interface.welding_command_handler:main',
        ],
    },
)
