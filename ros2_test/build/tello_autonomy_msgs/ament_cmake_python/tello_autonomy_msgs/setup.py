from setuptools import find_packages
from setuptools import setup

setup(
    name='tello_autonomy_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('tello_autonomy_msgs', 'tello_autonomy_msgs.*')),
)
