from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_perception_libs'],
    package_dir={'mdr_perception_libs': 'ros/src/mdr_perception_libs'}
)

setup(**d)