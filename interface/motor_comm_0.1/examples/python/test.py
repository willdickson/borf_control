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
    A = 40.0
    return A*scipy.cos(2.0*scipy.pi*t/T) 

print 'setting up interface'
comm = Motor_Comm()
comm.enable()
comm.standby('off')
comm.loop_mode('off')
dt = comm.dt()
num_motor = comm.num_motor()

print 'creating kinematics'
T = 2.5
num_period = 1.0
n = int(num_period*T/dt)

t = scipy.arange(0.0,n)*dt
x = scipy.zeros((n,num_motor))
for i in range(0,num_motor):
    x[:,i] = kine(t,T)

print 'loading outscan buffer'
comm.load_os_buffer(x)
#comm.print_status()

print 'moving to start'
comm.move2start()

print 'outscanning buffer'
comm.start()
comm.wait()

print 'reading buffers'
os_buff = comm.read_os_buffer()
ain_buff = comm.read_ain_buffer()

pylab.figure(1)
pylab.plot(t/T, os_buff[:,3], 'b')

pylab.figure(2)
pylab.plot(t/T, ain_buff[:,0], 'b')
    
pylab.show()
    
print 'moving to zero'
comm.move2zero()
comm.print_status()

print 'returning to standby mode'
comm.standby('on')







