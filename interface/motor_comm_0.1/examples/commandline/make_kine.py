#!/usr/bin/env python
import scipy
import pylab
from motor_comm import Motor_Comm

comm = Motor_Comm()
dt = comm.dt()
num_motor = comm.num_motor() 

T = 4.0
n = T/dt

t = scipy.arange(0,4*n)*dt
x = scipy.zeros((t.shape[0],num_motor))
x[:,0] = 400*scipy.cos(2.0*scipy.pi*t/T)+400*scipy.cos(4.0*scipy.pi*t/T)
x[:,1] = 500*scipy.cos(2.0*scipy.pi*t/T)+400*scipy.cos(4.0*scipy.pi*t/T) 
pylab.save('test_kine.txt', x)


