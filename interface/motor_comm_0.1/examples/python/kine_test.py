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
from BAI import BAI

# Constants
IND_MULT = 10.0
IND_PER_DEG = (7.2*4000.0/(360.0*IND_MULT))
DEG_PER_IND = 1.0/IND_PER_DEG
BAUDRATE = 38400
N_MOTOR = 2

# Kinematics functions

def cos_kine(t,T,A):
    A_ind = A*IND_PER_DEG
    return A_ind*scipy.cos(2.0*scipy.pi*t/T) 

def step_like(dt,t_start,t_total,step_size,step_dx):
    """
    """
    assert step_dx <= 1.0, 'step_dx > 1.0'
    step_duration = step_size*dt
    n = scipy.ceil(t_total/dt)
    t = scipy.arange(0.0,n)*dt
    x = scipy.zeros(t.shape)
    step_size_ind = step_size*IND_PER_DEG
    pos = 0.0
    for i, tval in enumerate(t):
        if tval <= t_start:
            continue
        if pos < step_size_ind:
            pos += step_dx 
        x[i] = pos
    return t,x

# Set up device interfaces 
print 'setting up motor-comm interface'
comm = Motor_Comm()
comm.loop_mode('off')
dt = comm.dt()
num_motor = comm.num_motor()
print 'opening serial interface'
bai = BAI(baudrate=BAUDRATE)

# Generate kinematics
if 0:
    # Cosine kinematics
    print 'creating cosine kinematics'
    A = 20.0
    T = 2.5 
    num_period = 2.0
    n = int(num_period*T/dt)
    t = scipy.arange(0.0,n)*dt
    x = scipy.zeros((n,num_motor))
    x[:,N_MOTOR] = cos_kine(t,T,A)
else:
    # Step function kinematics
    print 'creating step like kinematics'
    step_size = 2.0 
    step_dx = 0.1
    t_start = 1.0
    t_total = 4.0
    t,kine = step_like(dt,t_start,t_total,step_size,step_dx)
    x = scipy.zeros((kine.shape[0],num_motor))
    x[:,N_MOTOR] = kine 

    if 0:
        pylab.plot(t,kine)
        pylab.show()
        assert 1==0

# Load kinematics into outscan buffer and move to starting position
print 'loading outscan buffer'
comm.load_os_buffer(x)
print 'moving to start'
comm.move2start()

# Create storage arrays
t_list = []
pos_cmd_list = []
pos_tru_list = []

# Start outscan
print 'outscanning buffer'
comm.start()

# Wait for the outscan to start
while comm.outscan_status() == False:
    time.sleep(0.01)

# Grab data during outscan
t0 = time.time()
while comm.outscan_status() == True:
    t_list.append(time.time()-t0)
    pos_tru_list.append(bai.get_position()/IND_MULT)
    pos_cmd_list.append(comm.motor_ind()[N_MOTOR])
    time.sleep(0.001)

# Convert lists to arrays
t_array = scipy.array(t_list)
pos_cmd_array = scipy.array(pos_cmd_list)
pos_tru_array = scipy.array(pos_tru_list)

# Return to zero position and print status info
print 'moving to zero'
comm.move2zero()
comm.print_status()
print 'returning to standby mode'
comm.standby('on')

# Close serial interface
print 'closing serial interface'
bai.close()

# Plot results
pylab.plot(t_array, DEG_PER_IND*pos_cmd_array,'b')
pylab.plot(t_array, DEG_PER_IND*pos_tru_array,'r')
pylab.xlabel('time (sec)')
pylab.ylabel('position (deg)')
pylab.show()

