#!/usr/bin/env python
#
# A simple example demonstrating the use of the Motor_Comm class
# for communicating with the motor-ctl realtime driver
#
# Will Dickson 03/11/2008
# --------------------------------------------------------------
import scipy
import time
from motor_comm import Motor_Comm

print 'setting up interface'
comm = Motor_Comm()
comm.enable()
comm.standby('off')
comm.loop_mode('off')
dt = comm.dt()
num_motor = comm.num_motor()

num_trial = 200
for i in range(0,num_trial):
    
    T = 5.0 + scipy.rand()*(6.0-5.0)
    num_period = 10 #int(2.0 + scipy.rand()*(3.0 - 2.0))
    n = int(num_period*T/dt)
    A1 = scipy.rand()*(200.0 - 20.0)
    #A2 = 20.0 + scipy.rand()*(200.0 - 20.0)
    print 
    print '%d/%d) creating kinematics'%(i+1,num_trial)
    
    t = scipy.arange(0.0,n)*dt
    x = scipy.zeros((n,num_motor))
    
    x[:,0] = A1*scipy.cos(2.0*scipy.pi*t/T) #+ A2*scipy.cos(4.0*scipy.pi*t/T)
    x[:,2] = 30.0*scipy.cos(2.0*scipy.pi*t/T) #+ A2*scipy.cos(4.0*scipy.pi*t/T)

    print 'loading outscan buffer'
    comm.load_os_buffer(x)
    time.sleep(0.25)
    comm.print_status()

    print 'moving to start'
    comm.move2start()
    time.sleep(0.25)

    print 'outscanning buffer'
    comm.start()
    comm.wait()

    print 'reading buffers'
    os_buff = comm.read_os_buffer()
    ain_buff = comm.read_ain_buffer()
    print 'buffer shapes', os_buff.shape, ain_buff.shape
    
    print 'moving to zero'
    comm.move2zero()
    time.sleep(0.25)
    comm.print_status()

print 'returning to standby mode'
comm.standby('on')
