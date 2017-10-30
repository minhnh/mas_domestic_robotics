#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_lwr_kinematics'],
   package_dir={'mdr_lwr_kinematics': 'src/mdr_lwr_kinematics'}
)

setup(**d)
