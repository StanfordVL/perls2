#!/usr/bin/env python

import os
from setuptools import setup, find_packages

setup(
    name='perls2',
    version='0.0.1',
    install_requires=['gym'],  #And any other dependencies required
    include_package_data=True,
    description='PErception and Robotic Learning Stack',
    author='Stanford Vision and Learning Lab',
    author_email='rohun@stanford.edu',
    url='https://github.com/StanfordVL/perls2',
)
