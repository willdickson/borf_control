#!/usr/bin/env python
#
# A simple example demonstrating the use of the Motor_Comm class
# for communicating with the motor-ctl realtime driver
#
# Will Dickson 03/11/2008
# --------------------------------------------------------------
import scipy
import time
import pylab
from motor_comm import Motor_Comm

def kine(t,T):
    A = 100.0
    return A*scipy.cos(2.0*scipy.pi*t/T) #+ 100.0 - 150.0*scipy.cos(4.0*scipy.pi*t/T)

print 'setting up interface'
comm = Motor_Comm()
comm.enable()
comm.standby('off')
comm.loop_mode('on')
dt = comm.dt()
num_motor = comm.num_motor()

print 'creating kinematics'
T = 1.0
num_period = 1.0
n = int(num_period*T/dt)

t = scipy.arange(0.0,n)*dt
x = scipy.zeros((n,num_motor))
for i in range(0,num_motor):
    x[:,i] = kine(t,T)

print 'loading outscan buffer'
comm.load_os_buffer(x)

trig_ind = int(n/2)
print 'trig_ind:'
comm.set_trig_ind([0,1],[trig_ind,-1])

print 'moving to start'
comm.move2start()

print 'outscanning buffer'
comm.start()
    








