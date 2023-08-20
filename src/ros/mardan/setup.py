#!/usr/bin/env python

# see https://github.com/daniel-robotics/ros_python_pkg for more details

from os.path import dirname, abspath, basename
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=[
        'mardan',
        'mardan.hardware',
    ],
    package_dir={'': 'src'},
)

setup(**setup_args)
