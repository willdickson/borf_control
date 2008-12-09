"""
-----------------------------------------------------------------------
borf_control
Copyright (C) William Dickson, 2008.
  
wbd@caltech.edu
www.willdickson.com

Released under the LGPL Licence, Version 3

This file is part of borf_control.

borf_control is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.
    
borf_control is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with borf_control.  If not, see
<http://www.gnu.org/licenses/>.

------------------------------------------------------------------------
"""

from setuptools import setup, find_packages

setup(name='motor_comm',
      version='0.1',
      description='provides user-space communucation with motor_ctl hard real-time (RTAI) driver',
      author='William Dickson',
      author_email='wbd@caltech.edu',
      packages=find_packages(),
      entry_points = {'console_scripts' :['motor-comm=motor_comm:motor_comm_main',]}
     )
