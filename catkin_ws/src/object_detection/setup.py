#!/usr/bin/env python
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
# From: http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['sort', 'deeplab'],
    package_dir={'': 'include'},
    scripts=["script/sort_tracking.py", "script/yolo_detection.py"]
)

setup(**setup_args)