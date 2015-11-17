#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages = ['youbot_behavior_simple_test'],
    package_dir = {'': 'src'}
)

setup(**d)