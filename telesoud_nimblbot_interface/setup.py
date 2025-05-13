from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'telesoud_nimblbot_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Utiliser un chemin relatif simple
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thibault',
    maintainer_email='thibault@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telesoud_interface = telesoud_nimblbot_interface.telesoud_to_cartesian_command_interface:main',
            'welding_cartesian_command = telesoud_nimblbot_interface.welding_cartesian_command_for_telesoud:main',
        ],
    },
)
