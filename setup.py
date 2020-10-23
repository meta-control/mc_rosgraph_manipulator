#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mc_rosgraph_manipulator'],
   package_dir={'': 'src'}
)

setup(**d)
