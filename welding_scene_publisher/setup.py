from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'welding_scene_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scene_meshes'), glob('scene_meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thibault Courtois',
    maintainer_email='tcourtois@nimbl-bot.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'welding_scene_publisher = welding_scene_publisher.welding_scene_publisher:main'
        ],
    },
)
