#!/usr/bin/env python
from setuptools import setup, find_packages

setup(name='motor_comm',
      version='0.1',
      description='provides user-space communucation with motor_ctl hard real-time (RTAI) driver',
      author='William Dickson',
      author_email='wbd@caltech.edu',
      packages=find_packages(),
      entry_points = {'console_scripts' :['motor-comm=motor_comm:motor_comm_main',]}
     )
