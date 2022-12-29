#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['ocs2_python_interface'],
     package_dir={'': '/home/toto/Robotics2-project/src/ocs2'}
)

setup(**setup_args)
