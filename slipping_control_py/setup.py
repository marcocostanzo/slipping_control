#!/usr/bin/python
from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup

d = generate_distutils_setup(
    packages=['slipping_control_common'],
    package_dir={'': 'src'}
)

setup(**d)