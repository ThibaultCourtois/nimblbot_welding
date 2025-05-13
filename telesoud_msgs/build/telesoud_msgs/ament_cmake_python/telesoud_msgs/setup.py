from setuptools import find_packages
from setuptools import setup

setup(
    name='telesoud_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('telesoud_msgs', 'telesoud_msgs.*')),
)
