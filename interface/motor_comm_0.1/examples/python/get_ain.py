#!/usr/bin/env python
#
# A simple example demonstrating the use of the Motor_Comm class
# for communicating with the motor-ctl realtime driver
#
# Will Dickson 03/11/2008
# --------------------------------------------------------------
import sys
import scipy
import pylab
from motor_comm import Motor_Comm


n = int(sys.argv[1])
ain = scipy.zeros((n,))

comm = Motor_Comm()
for i in range(0,n):
    ain_data = comm.get_ain_data()
    ain[i] = ain_data[0]
    print i, ain_data

comm.close()
pylab.plot(ain)
pylab.show()
