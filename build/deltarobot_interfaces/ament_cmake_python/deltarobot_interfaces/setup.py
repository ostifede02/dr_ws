from setuptools import find_packages
from setuptools import setup

setup(
    name='deltarobot_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('deltarobot_interfaces', 'deltarobot_interfaces.*')),
)
