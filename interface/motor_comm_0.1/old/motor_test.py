#!/usr/bin/env python
import sys
import scipy
import ctypes 
import time
import matplotlib.pylab as pylab
from motor_shm import os_period_s
from motor_shm import load_os_buffer
from motor_shm import get_status_info
from motor_comm import cmd2motors


def get_ramp_move(x0, x1, velo, accel, dt):
    """
    Generates a point to point move from x0 to x1 with constant
    acceleration/de-acceleration (accel) to constant velocity (velo)
    with a step size of dt.
    """
    dist = abs(x1-x0)
    if dist == 0:
        # There is nothing to do 
        return scipy.array([[x0]],scipy.float)
    # Get the direction
    if x1-x0 > 0:
        a = abs(accel)
    else:
        a = -abs(accel)
    # Initialize position and velocity 
    x = x0
    v = 0.0
    traj_list = [x]
    #  Acceleration phase
    accel_cnt = 0
    while abs(v) <= abs(velo) and abs(x1-x) >= 0.5*dist:
        v = v + a*dt
        x = x + v*dt
        accel_cnt += 1
        traj_list.append(x)
    # Save acceleration distance
    accel_dist = abs(x0-x)
    # Constant velocty phase
    while abs(x1-x) >= accel_dist:
        x = x + v*dt
        traj_list.append(x)
    # De-acceleration phase
    for i in range(0,accel_cnt):
        v = v - a*dt
        x = x + v*dt
        traj_list.append(x)

    # Set last point of trajectory to x1
    traj_list[-1] = x1
    return scipy.array(traj_list)


def check_move(move):
    """
    Check that move doesn't contain any steps which are too
    large (>1). 
    """
    diff_move = move[1:] - move[:-1]
    max_step = max(scipy.absolute(diff_move))
    if max_step > 1:
        return False
    else:
        return True
    
    
def convert2index(move):
    """
    Converts continuous move to indices
    """
    move_index = scipy.around(move)
    return move_index.astype(scipy.integer)


# ---------------------------------------------------------------

kine_flag = 'trig'
plot_flag = False

# Generate ramp move in motor indices
print 'creating kimematics ... ',
sys.stdout.flush()
if kine_flag=='ramp':
    x0 = 0.0
    x1 = 1000.0
    velo = 1999.0
    accel = 2000.0
    dt = os_period_s()
    move = get_ramp_move(x0,x1,velo,accel,dt)
    t = scipy.arange(0,move.shape[0])*dt
    move0 = move
    move1 = move
    move2 = move
        
elif kine_flag=='trig':
    dt = os_period_s()    
    T = 2.5
    num_cycle = 1
    num_pt = T/dt
    t = scipy.arange(0,num_cycle*num_pt + 1)*dt
    move0 = 1*(100.0*scipy.cos(2.0*scipy.pi*t/T) + 100*scipy.cos(4.0*scipy.pi*t/T))
    move1 = 1*(100.0*scipy.cos(2.0*scipy.pi*t/T) + 100*scipy.cos(4.0*scipy.pi*t/T))
    move2 = 50.0*scipy.sin(2.0*scipy.pi*t/T) 

move0_index = convert2index(move0)
move1_index = convert2index(move1)
move2_index = convert2index(move2)

if plot_flag:
    pylab.subplot(311)
    pylab.plot(t,move0,'b')
    pylab.plot(t,move0_index,'r')
    pylab.ylabel('motor 0 index')
    pylab.subplot(312)
    pylab.plot(t,move1,'b')
    pylab.plot(t,move1_index,'r')
    pylab.ylabel('motor 1 index')
    pylab.subplot(313)
    pylab.plot(t,move2,'b')
    pylab.plot(t,move2_index,'r')
    pylab.ylabel('motor 1 index')
    pylab.xlabel('t(sec)')
    pylab.show()
    sys.exit(0)
    
move0_index = move0_index-move0_index[0]
move1_index = move1_index-move1_index[0]
move2_index = move2_index-move2_index[0]
check_flag0 = check_move(move0_index)
check_flag1 = check_move(move1_index)
check_flag2 = check_move(move2_index)

print 'done'
print 'check flag0:',check_flag0,'check_flag1:', check_flag1, 'check_flag2:', check_flag2
if (not check_flag0) or (not check_flag1) or (not check_flag2):
    raise ValueError

# Pack into 2d array - kludge
N = move0.shape[0]
move_index = scipy.zeros((N,3),scipy.integer)
move_index[:,0] = move0_index
move_index[:,1] = move1_index
move_index[:,2] = move2_index
move_index = scipy.ascontiguousarray(move_index)
    
# Load data into buffer
print 'loading data into buffer ...',
sys.stdout.flush()
cmd2motors('unlock-buffer')
time.sleep(0.1) # Note a good way to do tings (really want a try loop)
flag = load_os_buffer(move_index)
cmd2motors('lock-buffer')
print 'done'
    







