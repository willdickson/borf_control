# motor_comm.py
#
# Set of routines which provide commandline based interface to the
# motor_ctl hard real-time (RTAI) driver.
#
# William Dickson 01/28/2008
# ---------------------------------------------------------------------
import sys
import struct
import time
import optparse
import scipy
import atexit
from motor_shm import cmd_fifo
from motor_shm import cmd_dict
from motor_shm import get_status_info
from motor_shm import load_buffer
from motor_shm import read_os_buffer
from motor_shm import read_ain_buffer
from motor_shm import mv_buffer
from motor_shm import os_buffer
from motor_shm import shm_alloc
from motor_shm import shm_free

DEBUG=True

NSEC2SEC = 1.0e-9
DFLT_MOVE_ACCEL = 100.0 #steps/sec**2
DFLT_MOVE_MAX_VELO = 100.0 #steps/sec
DFLT_WAIT_SLEEP_T = 0.5
BUFFER_LOCK_MAX_CNT = 5
BUFFER_LOCK_SLEEP_T = 0.25
BUFFER_SWAP_SLEEP_T = 0.25
BUFFER_SWAP_MAX_CNT = 10


def debug_print(msg):
    if DEBUG==True:
        print '\t  *%s'%(msg,)

def cmd2motor_ctl(cmd,*args):
    """
    Sends commands to motor via motor_ctl command FIFO.

    commands                args 
    ---------------------------------------------------
    outscan-off             none
    outscan-on              none
    standby-on              none
    standby-off             none
    enable                  none
    disable                 none
    unlock-buffer           none
    lock-buffer             none
    zero-buffer-pos         none
    zero-motor-ind          none
    loop-mode-on            none
    loop-mode-off           none
    set-trig-ind            trigger #, trig index
    set-trig-wid            trigger #, trigger width
    set-os-buffer           none
    set-mv-buffer           none
    """
    debug_print('cmd2motor_ctl %s'%(cmd,))
    cmd_data = cmd_dict[cmd.lower()] 
    cmd_num = cmd_data[0]
    num_arg = cmd_data[1]
    fid = open(cmd_fifo, 'wb')
    if num_arg == 0:
        out_str = struct.pack('i',cmd_num)
    if num_arg == 2:
        out_str = struct.pack('iii',cmd_num,args[0],args[1])
        
    fid.write(out_str)
    cnt = 0
    max_cnt = 5
    closed = False
    while not closed:
        try:
            fid.close()
            closed = True
        except IOError:
            cnt+=1
            if cnt == max_cnt:
                raise IOError, 'unable to close fifo'

def get_ramp_moves(p0,p1,vmax,a,dt):
    """
    Generate ramp trajectories from vector p0 to vector p1. 
    
    Arguments:
      p0 = vector of starting points
      p1 = vector of ending points
      vmax = maximum allowed velocity
      a = constant acceleration
      
    Output:
      ramp_array = array of ramp trajectories

    """
    debug_print('get_ramp_moves')
    ramp_list = []
    for x0,x1 in zip(p0,p1):
        ramp = get_ramp(x0,x1,vmax,a,dt)
        ramp_list.append(ramp)
        
    # Make all ramps the same length by padding the short ones
    # Reshape at the same time so that all ramps are (maxlen,1)
    max_len = max(r.shape[0] for r in ramp_list)        
    for i,r in enumerate(ramp_list):
        if r.shape[0] < max_len:
            pad = r[-1]*scipy.ones((max_len-r.shape[0],))
            r = scipy.hstack((r,pad))
        ramp_list[i] = r.reshape((max_len,1))

    ramp_array = scipy.hstack(ramp_list)
    return ramp_array
        
def get_ramp(x0,x1,vmax,a,dt, output='ramp only'):
    """
    Generate a ramp trajectory from x0 to x1 with constant
    acceleration, a, to maximum velocity v_max. 

    Note, the main purlpose of this routine is to generate a
    trajectory from x0 to x1. For this reason v_max and a are adjusted
    slightly to work with the given time step.

    Arguments:
     x0 = starting position
     x1 = ending position
     vmax = maximum allowed velocity
     a = constant acceleration
     
     Keywords:
       output = 'ramp only' or 'full'
       when ramp only is selected then only the velocity ramp is returned. 
       If 'full' is selected the adjusted acceleration and maximum velocity 
       are also returned.
       
    Ouput:
      ramp = ramp trajectory form x0 to x1


    """
    debug_print('get_ramp')
    # Insure we are dealing with floating point numbers
    x0, x1 = float(x0), float(x1)
    vmax, a = float(vmax), float(a)
    dt = float(dt)
    vmax, a = abs(vmax), abs(a) # Make sure that v_max and a are positive

    # Check to see if there is anything to do
    if x0==x1:
        return scipy.array([x0])

    # Get distance and sign indicating direction
    dist = abs(x1-x0)
    sign = scipy.sign(x1-x0)

    # Determine if we will reach v_max
    t2vmax = vmax/a
    t2halfdist = scipy.sqrt(0.5*dist/a)
    
    if t2vmax > t2halfdist:
        # Trajectory w/o constant velocity segment  
        T = scipy.sqrt(dist/a)
        n = int(scipy.round_((1.0/dt)*T))
         
        # Adjust accel and duration for rounding of n (discrete time steps)
        a = dist/(n*dt)**2
        T = scipy.sqrt(dist/a)
        
        # Generate trajectory
        t = scipy.linspace(0.0,2.0*T,2*n+1)
        def f1(t):
            return 0.5*sign*a*(t**2)
        def f2(t):
            s = t-T
            return f1(T)+ sign*a*T*s - 0.5*sign*a*s**2
        func_list = [f1,f2]
        cond_list = [t<=T, t>T]
        ramp = x0+scipy.piecewise(t,cond_list,func_list)
          
    else:
        # Trajectory w/ constant velocity segment
        # Compute acceleration time and adjust acceleration
        T1 = vmax/a 
        n = int(scipy.round_(T1/dt))
        a = vmax/(n*dt) # Adjusted acceleration 
        T1 = vmax/a # Adjusted acceleration time  

        # Compute and adjust constant velocity time
        T2 = dist/vmax - T1  
        m = int(scipy.round_(T2/dt))
        vmax = dist/(dt*(n+m)) # Adjusted max velocity  
        T2 = dist/vmax - T1 # Adjusted constant velocity time

        # Generate trajectory
        t = scipy.linspace(0.0,2.0*T1+T2,2*n+m+1)
        def f1(t):
            return 0.5*sign*a*(t**2)
        def f2(t):
            s = t-T1
            return f1(T1) + sign*vmax*s
        def f3(t):
            s = t-T1-T2
            return f2(T1+T2)+sign*vmax*s-0.5*sign*a*s**2 
        func_list = [f1,f2,f3]
        cond_list = [t<=T1, scipy.logical_and(t>T1,t<=T1+T2), t>T1+T2]
        ramp = x0+scipy.piecewise(t,cond_list,func_list)

    if output=='ramp only':
        return ramp
    elif output=='full':
        return ramp, vmax, a
    else:
        raise ValueError, 'unknown keyword option output=%s'%(output,)


def check_moves(buff_data):
    """
    Check that moves don't contain steps which are too large (>1).
    Returns a test flag which is true if all moves pass and false
    otherwise and a list of the columns of buff_data which failed.
    """
    debug_print('check_moves')
    m = buff_data.shape[1]
    test_flag = True
    failed_moves = []
    for i in range(0,m):
        move = buff_data[:,i]
        if move.shape[0]==1:
            continue
        diff_move = move[1:]-move[:-1]
        max_step = scipy.absolute(diff_move).max()
        if max_step > 1:
            test_flag = False
            failed_moves.append(i)     
    return test_flag, failed_moves
    
def convert2int(move):
    """
    Converts continuous move to indices
    """
    debug_print('convert2int')
    move_index = scipy.around(move)
    return move_index.astype(scipy.integer)


# ---------------------------------------------------------
# Python interface
class Motor_Comm:
    """
    Defines an interface w/ the motor_ctl hard real-time driver. 
    """
    def __init__(self):
        """
        Initialize Motor_Ctl interface
        """
        self.debug=DEBUG
        self.debug_print('__init__\n')
        #self.open = True
        self.move_max_velo = DFLT_MOVE_MAX_VELO
        self.move_accel = DFLT_MOVE_ACCEL
        self.wait_sleep_t = DFLT_WAIT_SLEEP_T
        flag = shm_alloc() # Allocate shared memory w/ motor_ctl
        if flag != 0:
            raise IOError, 'unable to allocate shared memory'
        else:
            self.shm = True
            atexit.register(self.atexit)

    def debug_print(self,msg):
        if self.debug==True:
            print '\t*%s'%(msg,)

    def atexit(self):
        self.debug_print('atexit')
        if self.shm == True:
            shm_free()
            print 'freeing shared memory in atexit'
            self.shm=False

    def __del__(self):
        self.debug_print('__del__')
        if self.shm==True:
            shm_free() 
            print 'freeing shared memory in __del__'
            self.shm=False        
        
    def set_os_buffer(self):
        self.debug_print('set_os_buffer')
        cmd2motor_ctl('set-os-buffer')
        cnt, test = 0, False
        while cnt < BUFFER_SWAP_MAX_CNT:
            time.sleep(BUFFER_SWAP_SLEEP_T)
            if self.get_cur_buffer() == 'os_buffer':
                test = True
                break
            print '\t', cnt, self.status()
            cnt+=1
        if test==False:
            raise IOError, 'unable to set os_buffer'

    def set_mv_buffer(self):
        self.debug_print('set mv_buffer')
        cmd2motor_ctl('set-mv-buffer')
        cnt, test = 0, False
        while cnt < BUFFER_SWAP_MAX_CNT:
            time.sleep(BUFFER_SWAP_SLEEP_T)
            if self.get_cur_buffer() == 'mv_buffer':
                test = True
                break
            print '\t', cnt, self.get_cur_buffer()
            cnt+=1
        if test==False:
            raise IOError, 'unable to set mv_buffer'
        

    def get_cur_buffer(self):
        self.debug_print('get_cur_buffer')
        status = self.status()
        if status['buffer'] == os_buffer():
            cur_buffer = 'os_buffer'
        else:
            cur_buffer = 'mv_buffer'
        return cur_buffer
         
    def motor_ind(self):
        """
        Returns current motor indices
        """
        self.debug_print('motor_ind')
        status = self.status()
        return status['motor_ind']

    def buffer_cur_len(self):
        """
        Returns the length of the current outscan buffer.
        """
        self.debug_print('buffer_cur_len')
        status = self.status()
        return status['buffer_cur_len']
        #return get_os_buffer_len()

    def buffer_pos(self):
        """
        Returns current position in the outscan buffer.
        """
        self.debug_print('buffer_pos')
        status = self.status()
        if self.cur_buffer() == 'os_buffer':
            pos = status['os_buffer_pos']
        else:
            pos = status['mv_buffer_pos']
        return pos

    def dt(self):
        self.debug_print('dt')
        """
        Return the perid of the real-time loop.
        """
        return 1.0/self.freq()

    def freq(self):
        self.debug_print('freq')
        """
        Return the frequency of the real-time loop.
        """
        status = self.status()
        return status['frequency']

    def disable(self):
        self.debug_print('disable')
        """
        Disable stepper motors
        """
        cmd2motor_ctl('disable')

    def enable(self):
        """
        Enable stepper motors
        """
        cmd2motor_ctl('enable')

    def is_enabled(self):
        """
        Returns True if stepper motors are enabled and False otherwise.
        """
        self.debug_print('is_enabled')
        status = self.status()
        if status['enable']==1:
            val = True
        else:
            val = False
        return val

    def is_standby(self):
        """
        Returns True of motors are in standby mode and False otherwise.
        """
        self.debug_print('is_standby')
        status = self.status()
        if status['standby']==1:
            val = True
        else:
            val = False
        return val

    def get_pwm_zeros(self):
        """
        Get the pwm zeros for all motors in seconds.
        """
        self.debug_print('get_pwm_zeros')
        status = self.status()
        pwm_zero_ns = status['pwm_zero_ns']
        return [NSEC2SEC*x for x in pwm_zero_ns]

    def num_motor(self):
        """
        Returns the number of motors (pwm+stepper)
        """
        self.debug_print('num_motor')
        status = self.status()
        return len(status['motor_ind'])

    def buffer_max_len(self):
        """
        Returns the maximum allowed outscan buffer length
        """
        self.debug_print('buffer_max_len')
        status = self.status()
        return status['buffer_max_len']

    def load_os_buffer(self,buff_data):
        """
        Load data into os_buffer. Automatically sets buffer to
        os_buffer
                
        Argument:

          buff = buffer name 'mv_buffer' or 'os_buffer'
          buff_data = NxNUM_MOTOR array of motor indices.

        """
        self.debug_print('load_os_buffer')
        self.load_buffer('os_buffer', buff_data)

    def load_mv_buffer(self,buff_data):
        """
        Load data into mv_buffer. Automatically sets buffer to
        mv_buffer
        """
        self.debug_print('load_mv_buffer')
        self.load_buffer('mv_buffer', buff_data)

    def load_buffer(self,buff,buff_data):
        """
        Load data into buffer. Automatically sets buffer to buff.

        Argument:

          buff = buffer name 'mv_buffer' or 'os_buffer'
          buff_data = NxNUM_MOTOR array of motor indices.
        """
        self.debug_print('load_buffer')
        if buff_data == None:
            raise ValueError, 'buff_data must be an array'
        # Check buffer shape
        n,m = buff_data.shape
        num_motor = self.num_motor()
        buffer_max_len = self.buffer_max_len()
        if m!=num_motor or n>buffer_max_len:
            msg_tuple = (n,m,num_motor,buffer_max_len)
            msg = 'incorrect buffer shape (%d,%d) - should be Nx%d where N<=%d'%msg_tuple
            raise ValueError, msg
        
        # Convert to integers and check 
        idata = convert2int(buff_data)
        test_flag, failed_moves = check_moves(idata)
        if test_flag==False:
            msg = 'columns %s in outscan buffer contain steps  > 1'%(str(failed_moves),)
            raise ValueError, msg

        # Load data into outscan buffer
        self.unlock_buffer()
        if buff == 'os_buffer':
            self.set_os_buffer()
        elif buff == 'mv_buffer':
            self.set_mv_buffer()
        else:
            raise ValueError, 'unknown buffer'
        load_buffer(buff,idata)
        self.lock_buffer()

    def os_buffer_start(self):
        self.debug_print('os_buffer_start')
        status = self.status()
        return status['os_buffer_start']

    def read_os_buffer(self,mode='full'):
        """
        Read contents of outscan buffer

        Keyword: mode = 'full' or '1st line'. If mode is 'full' the
        contents of the entire outscan buffer are returned. If mode is
        '1st line' only the first line of the outscan buffer is
        returned.
        """
        self.debug_print('read_os_buffer')
        if not mode in ('full', '1st line'):
            raise ValueError, 'Unknown mode %s'%(mode,)
        os_buff = read_os_buffer(mode=mode)
        return os_buff
      
    def read_ain_buffer(self):
        """
        Read the contents of the analog input buffer.
        """
        self.debug_print('read_ain_buffer')
        ain_buff = read_ain_buffer()
        return ain_buff

    def buffer_is_locked(self):
        """
        Returns True is the outscan buffer is locked and False if it
        is not.
        """
        self.debug_print('buffer_is_locked')
        status = self.status()
        if status['buffer_lock']==1:
            val = True
        else:
            val = False
        return val
        
    def lock_buffer(self):
        """
        Locks the outscan buffer
        """
        self.debug_print('lock_buffer')
        cmd2motor_ctl('lock-buffer')
        cnt = 0
        test_flag = False
        while cnt < BUFFER_LOCK_MAX_CNT:
            time.sleep(BUFFER_LOCK_SLEEP_T)
            if self.buffer_is_locked()==False:
                print 'warning: locking - buffer unlocked test, cnt=%d'%(cnt,)
                continue
            else:
                test_flag = True
                break
        if test_flag==False:
            raise IOError, 'unable to lock outscan buffer'


    def unlock_buffer(self):
        """
        Unlocks the outscan buffer
        """
        self.debug_print('unlock_buffer')
        cmd2motor_ctl('unlock-buffer')
        cnt = 0
        test_flag = False
        while cnt < BUFFER_LOCK_MAX_CNT:
            time.sleep(BUFFER_LOCK_SLEEP_T)
            if self.buffer_is_locked()==True:
                print 'warning: unlocking  - buffer locked test, cnt=%d'%(cnt,)
                continue
            else:
                test_flag = True
                break
        if test_flag==False:
            raise IOError, 'unable to unlock outscan buffer'
            
    def standby(self,val):
        """
        set standby  mode ('on' or 'off').
        """
        self.debug_print('standby %s'%(val,))
        if val.lower()=='on':
            cmd2motor_ctl('standby-on')
        elif val.lower()=='off':
            cmd2motor_ctl('standby-off')
        else:
            raise ValueError, 'unkown value for standby'
        return

    def loop_mode(self,val):
        """
        set  loop mode ('on' or 'off').
        """
        self.debug_print('loop_mode %s'%(val,))
        if val.lower()=='on':
            cmd2motor_ctl('loop-mode-on')
        elif val.lower()=='off':
            cmd2motor_ctl('loop-mode-off')
        else:
            raise ValueError, 'unkown value for loop mode'
        return

    def get_loop_mode(self):
        if self.is_loop_mode():
            return 'on'
        else:
            return 'off'

    def is_loop_mode(self):
        """
        Returns True if loop mode is on and False if it is not.
        """
        self.debug_print('is_loop_mode')
        status = self.status()
        if status['loop_mode']==1:
            return True
        else:
            return False

    def move2start(self):
        """
        Moves to start of current outscan buffer. Blocks until move is
        completed.
        """
        self.debug_print('move2start')
        pos = self.os_buffer_start()
        if pos==None:
            return
        else:
            self.move2pos(pos)
        
    def moveby(self,moves):
        """
        Moves motors by amounts the amounts specified - relative
        moves. Blocks until move is completed. Note, this command is
        the building block of all other move commands: move2pos,
        move2zero, etc.

        Argument:

          moves = a tuple, list or array of moves given in motor indices
        
        """
        self.debug_print('moveby')
        # Get currnt state 
        loop_mode = self.get_loop_mode()
        cur_buffer = self.get_cur_buffer()
        
        # Get ramp moves
        moves=scipy.array(moves)
        p0 = scipy.zeros(moves.shape[0])
        p1 = moves
        ramps = get_ramp_moves(p0,p1,self.move_max_velo,self.move_accel,self.dt())
    
        # Turn off loop mode and load trajectories
        self.loop_mode('off')
        self.load_mv_buffer(ramps)
        
        # Start and wait for outscan
        self.start()
        self.wait()

        # Return to previous state
        self.loop_mode(loop_mode)
        if cur_buffer == 'os_buffer':
            self.unlock_buffer()
            self.set_os_buffer()
            self.lock_buffer()
                
    def move2pos(self,pos):
        """
        Move motors to specified index positions - absolute
        moves. Blocks until move is completed.

        Argument:
          pos = a tuple, list or array of motor positions given in
          motor indices.
        """
        self.debug_print('move2pos')
        pos = scipy.array(pos)
        cur_pos = scipy.array(self.motor_ind())
        moves = pos-cur_pos
        self.moveby(moves)
        
    def move2zero(self):
        """
        Move motors to the zero index position. Blocks until move is
        completed,
        """
        self.debug_print('move2zero')
        pos = scipy.zeros((self.num_motor(),))
        self.move2pos(pos)
        return
        
    def start(self):
        """
        Start outscan
        """
        self.debug_print('start')
        cmd2motor_ctl('outscan-on')
        

    def stop(self):
        """
        Stop outscan. Nonblocking.
        """
        self.debug_print('stop')
        cmd2motor_ctl('outscan-off')

    def outscan_status(self):
        """
        Returns True if an outscan is in progress and False otherwise.
        """
        self.debug_print('outscan_status')
        status = self.status()
        outscan_flag = status['outscan']
        if outscan_flag==1:
            return True
        else:
            return False

    def wait(self):
        """
        Block until outscan stops. Note, in loop mode this will wait forever
        """
        self.debug_print('wait')
        while 1:
            time.sleep(self.wait_sleep_t)
            if self.outscan_status()==False:
                break
        return
        
    def status(self):
        """
        Get status information
        """
        self.debug_print('status')
        status = get_status_info()
        return status

    def zero_buffer_pos(self):
        """
        Set position in outscan buffer to zero
        """
        self.debug_print('zero_buffer_pos')
        cmd2motor('zero-buffer-pos')
        return

    def zero_motor_ind(self):
        """
        Set the current motor position to the zero index positon. 
        """
        self.debug_print('zero_motor_ind')
        cmd2motor('zero-motor-ind')
        return

    def set_trig_ind(self, trig_num, trig_ind):
        """
        sets the outscan index positions at which the given triggers
        will fire during an outscan. A indes value of -1 turns of the
        given trigger.
        """
        self.debug_print('set_trug_ind')
        for n,i in zip(trig_num, trig_ind):
            cmd2motor_ctl('set-trig-ind',n,i)
        return

    def set_trig_wid(self, trig_num, trig_wid):
        """
        sets the width (in outscan indices) the given triggers.
        This command requires two additional arguments: the trigger numbers
        """
        self.debug_print('set_trig_wid')
        for n,i in zip(trig_num, trig_wid):
            cmd2motor_ctl('set-trig-wid',n,i)
        return

    def get_trig_ind(self):
        """
        Returns list current trigger indices
        """
        self.debug_print('get_trig_ind')
        status = self.status()
        return status['trig_index']
    
    def get_trig_wid(self):
        """
        Returns list of current trigger widths
        """
        self.debug_print('get_trig_wid')
        status = self.status()
        return status['trig_width']
        
    def cmd2motor_ctl(self, cmd_str, *args):
        """
        Sends of cmd to motor_ctl driver via the cmd2motor function.
        """
        self.debug_print('cmd2motor_ctl')
        cmd2motor_ctl(cmd_str, *args)
        return
            
    def print_status(self):
        """
        Display status information
        """
        self.debug_print('print_status')
        status = self.status()
        status_keys = status.keys()
        status_keys.sort()
        for key in status_keys:
            print '  %s:'%(key,), status[key]
        

    def get_ain_data(self):
        """
        Get immediate value of analog input data from status 
        shared memory
        """
        debug_print('get_ain_data')
        status = self.status()
        ain_data = status['ain_data']
        return ain_data


# ----------------------------------------------------------
# Commandline utility

usage =""" %prog command [OPTION]...

%prog is command line utility for interfacing the motor_ctl hard real-time driver.

Command Summary:

 disable         - disbale stepper motor drive
 enable          - enable stepper motor drive
 help-cmd        - get help on specific command
 help            - print this message
 load-buffer     - load kinematics into outscan buffer 
 lock-buffer     - lock the ourscan buffer for outscan
 loop-mode-on    - turn on loop mode, outscan will loop forever
 loop-mode-off   - turn off loop mode, outscan will repeat once
 moveby          - move motors by specfied amounts 
 move2pos        - move motors to specified position
 move2start      - move to start of current outscan buffer
 move2zero       - move motors to zero position
 outscan-off     - turn off outscan     
 outscan-on      - turn on outscan
 read-buffer     - reads contents of outscan buffer and returns on stdout
 set-trig-ind    - set outscan index position of triggers 
 set-trig-wid    - set length of trigger in outscan indices 
 standby-on      - turn on standby mode for stepper motors    
 standby-off     - turn off standby mode for stepper motors     
 start           - start outscan
 stop            - stop outscan
 status          - print status information
 unlock-buffer   - unlocks the outscan buffer for writing  
 zero-buffer-pos - set position in outscan buffer to zero
 zero-motor-ind  - set motor index position to zero
  
Examples: %prog help-cmd load-buffer 
          %prog loop-mode-on
          %prog start
          %prog status
          %prog load-buffer kine.txt
          %prog moveby 10 15 20 12 10
          %prog set-trig-ind 0 1000
          %prog set-trig-wid '0 1 2 3' '1000 1100 2000 2025'
"""

outscan_off_help = """
outscan-off: immediately turns off the current outscan. The buffer
position is not reset to zero and the outscan may be continued from
where it left off using the outscan-on command.

Example: motor-comm outscan-off
"""

outscan_on_help ="""
outscan-on: turns on the outscan. Starting from the current buffer
position the position commands in the current outscan buffer will be
sent to the motors at clock rate.

Example: motor-comm outscan-on

"""
standby_on_help = """
standby-on: sets stepper motors to standby mode - a low power state
used for position holding to avoid over heating the motors.

Example: motor-comm standby-on
"""

standby_off_help = """
standby-off: removes stepper motors from standby mode and returns them
to normal high torque operating mode.

Example: motor-comm standby-off
"""

enable_help ="""
enable: enables stepper motor drive. When enabled power is sent to
steppers and the motors will have holding torque.

Example: motor-comm enable
"""
disable_help ="""
disable: disables stepper motor drive. When disbaled no power is sent to
steppers and motors will not have holding torque.

Example: motor-comm disable
"""

unlock_buffer_help = """
unlock-buffer: unlocks outscan buffer for writing. It is required that
the outscan buffer be unlocked before writing.  Note, that the outscan
buffer cannot be unlocked for writing when an outscan is in
progress. This is a low level command and in general it is not
necessary to manually unlock and lock the outscan buffer as the
load_buffer command unlocks, writes to, and then relocks the outscan
buffer.

Example: motor-comm unlock
"""

lock_buffer_help = """
lock-buffer: locks outscan buffer. It is required that the outscan
buffer be locked before starting an outscan.This is a low level
command and in general it is not necessary to manually unlock and lock
the outscan buffer as the load_buffer command unlocks, writes to, and
then relocks the outscan buffer.

Example: motor-ctl lock
"""

zero_buffer_pos_help = """
zero-buffer-pos: sets the current position in the outscan buffer to zero.

Example: motor-ctl zero-buffer-pos
"""

zero_motor_ind_help = """
zero-motor-ind: sets the curent motor index position to zero

Example: motor-comm zero-motor-ind
"""

loop_mode_on_help = """
loop-mode-on: turns on loop mode. Normally an outscan will run from
the beginning to the end of the outscan buffer and then stop. When
loop mode is set to on upon reaching the end of the outscan buffer the
buffer position index will be reset to zero and the outscan will
continue from the begining of the buffer. The outscan will loop
indefinitely until the the loop mode is turned off or the outscan is
stopped.

Example: motor-comm loop-mode-on
"""

loop_mode_off_help = """
loop-mode-off turns off loop mode. Normally an outscan will run from
the beginning to the end of the outscan buffer and then stop. When
loop mode is set to on upon reaching the end of the outscan buffer the
buffer position index will be reset to zero and the outscan will
continue from the begining of the buffer. The outscan will loop
indefinitely until the the loop mode is turned off or the outscan is
stopped.

Example: motor-comm loop-mode-off
"""

set_trig_ind_help = """
set-trig-ind: sets the outscan index positions at which the given
triggers will fire during an outscan. This command requires two
additional arguments: the trigger numbers and the indices at which
they should fire. There must be the same number of triggers as indices.
Setting the index position to -1 turns off the the trigger.

Usage: motor-comm set-trig-ind triggers indices

 triggers = trigger numbers
 indices  = indices at which triggers should fire

Examples: # set trigger 0 to fire at index 121
          motor-comm set-trig-ind 0 121

          # set trigger 0,1,2 to fire at indices 212, 300, 500
          motor-comm set-trig-ind '0 1 2' '212 300 500' 
"""

set_trig_wid_help = """
set-trig-wid: sets the width (in outscan indices) the given triggers.
This command requires two additional arguments: the trigger numbers
and the widths to which they should be set. Each outscan index has a
duration of 1/(frequency) where frequency is the frequency of the
real-time loop.

Usage: motor-comm set-trig-wid triggers widths

 triggers = trigger numbers
 widths  =  widths (in indices) for each trigger 

Examples: # set trigger 0 to a width of 10
          motor-comm set-trig-wid 0 121

          # set trigger 0,1 and 2 to widths 10, 30 and 50
          motor-comm set-trig-wid '0 1 2' '212 300 500' 
"""
load_buffer_help = """
load-buffer: loads motor kinematics into outscan buffer. This command
unlocks the outscan buffer, writes kinematics to the buffer and then
relocks the buffer. One additional argument is required, a the file
containing the motor kinematics. 

Usage: motor-ctl load-buffer kine_file
  
  kine_file = file containing motor kinematics. Should be text file
  with N rows and NUM_MOTOR columns of motor position indices. Entry
  (i,j) specifies the desired position of motor j at outscan index i. 


Examples: motor-comm load-buffer kine_file.txt
          motor-comm load-buffer /some_dir/some-sub_dir/kine_file.txt
"""

moveby_help = """
moveby: generates a relative move which moves each motor by the amount
specified. Requires NUM_MOTOR additional arguments. Note, negative
move values are denoted using an 'n' in front of the number instead of
a '-' sign, e.g. -100 is given as n100. Positive move values can be
given by just the number or using a 'p' in front of the number, e.g.,
100 or p100. Note, the command does not return until the move is
completed.

Usage: motor-comm moveby move_0,..., move_NUM_MOTORS

  move_0 =  amount (in indices) to move motor 0
   ...
  move_NUM_MOTORS = amount (in indices) to move motor NUM_MOTORS


Examples: motor-comm moveby n10 15 10 n30 45
"""

move2zero_help = """
move2zero: moves all motor to the zero position - specified in motor
indices. Note, this command does not return until the move is
completed.

Example: motor-comm move2zero
"""

move2pos_help = """
move2pos: moves all motors to the specified positions. Requires
NUM_MOTOR additional arguments specifying the desired position of the
motors in indices. Note, negative position values are denoted using an
'n' in front of the number instead of a '-' sign, e.g. -100 is given
as n100. Positive position values can be given by just the number or
using a 'p' in front of the number, e.g., 100 or p100. Note, this
command does not return until the move is completed.


Usage: motor-comm move2pos move_0,..., move_NUM_MOTORS

  move_0 =  amount (in indices) to move motor 0
   ...
  move_NUM_MOTORS = amount (in indices) to move motor NUM_MOTORS


Example: motor-comm move2pos 100 100 n100 20 0
"""

move2start_help = """
move2start: moves all motors to the start of the current outscan
buffer. Note, this command does not return until the move is
completed.

Example: motor-comm move2start
"""

start_help = """
start: starts an outscan. Note the outscan is run relative to the
current position. 

Example: motor-comm start
"""

stop_help = """
stop: stops outscan.

Example: motor-comm stop
"""

status_help = """
status: print status information for the motor_ctl driver.

Example: motor-comm status
"""

help_cmd_help = """
help-cmd: print help messages for specified command. Requires 1
additional argument.

Usage: motor-comm help some-command

  some-command = a motor-comm command

Examples: motor-comm help-cmd status
          motor-comm help-cmd load-buffer
"""

help_help = """
help: displays motor-ctl help message giving usage information and a
summary of the basic commands.

Example: motor-ctl help
"""

read_buffer_help = """
read-buffer: reads the contents of the current outscan buffer and
returns the values via stdout.

Examples: motor-comm read-buffer
          motor-comm read-buffer > somefile.txt 
"""

class Motor_Comm_Cmd_Line:

    def __init__(self):

        self.cmd_table = {
            'outscan-off' : self.cmd_no_args,
            'outscan-on' : self.cmd_no_args, 
            'standby-on' : self.cmd_no_args, 
            'standby-off' : self.cmd_no_args, 
            'enable' : self.cmd_no_args, 
            'disable' : self.cmd_no_args, 
            'unlock-buffer' : self.cmd_no_args,    
            'lock-buffer' : self.cmd_no_args,      
            'zero-buffer-pos' : self.cmd_no_args,  
            'zero-motor-ind': self.cmd_no_args,   
            'loop-mode-on': self.cmd_no_args,    
            'loop-mode-off': self.cmd_no_args,   
            'set-trig-ind' : self.set_trig_ind,    
            'set-trig-wid': self.set_trig_wid,    
            'load-buffer' : self.load_buffer, 
            'moveby': self.moveby,     
            'move2zero': self.move2zero, 
            'move2pos' : self.move2pos,
            'move2start' : self.move2start,
            'start' : self.start,
            'stop' : self.stop,
            'status' : self.status,
            'help-cmd' : self.help_cmd,
            'help' : self.help,
            'read-buffer': self.read_buffer,
            }
        
        self.help_table = {
            'outscan-off' : outscan_off_help,
            'outscan-on' : outscan_on_help, 
            'standby-on' : standby_on_help, 
            'standby-off' : standby_off_help, 
            'enable' : enable_help, 
            'disable' : disable_help, 
            'unlock-buffer' : unlock_buffer_help,    
            'lock-buffer' : lock_buffer_help,      
            'zero-buffer-pos' : zero_buffer_pos_help,  
            'zero-motor-ind': zero_motor_ind_help,   
            'loop-mode-on': loop_mode_on_help,    
            'loop-mode-off': loop_mode_off_help,   
            'set-trig-ind' : set_trig_ind_help,    
            'set-trig-wid': set_trig_wid_help,    
            'load-buffer' : load_buffer_help, 
            'moveby': moveby_help,     
            'move2zero': move2zero_help, 
            'move2pos' : move2pos_help,
            'move2start' : move2start_help,
            'start' : start_help,
            'stop' : stop_help,
            'status' : status_help,
            'help-cmd' : help_cmd_help,
            'help': help_help,
            'read-buffer': read_buffer_help,
            }
        

        parser = optparse.OptionParser(usage=usage)
    
        parser.add_option('-v', '--verbose',
                          action='store_true',
                          dest ='verbose',
                          help='verbose mode - print additional information',
                          default = False
                          )

        parser.add_option('-w', '--wait',
                          action='store_true',
                          dest ='wait',
                          help='wait for outscan to complete before returning',
                          default = False
                          )
 
        self.options, self.args = parser.parse_args()
        self.parser = parser
        self.motor_comm = Motor_Comm()
        return

    def run_command(self):
        try:
            cmd_str = self.args[0]
        except:
            print 'E: no command argument'
            sys.exit(1)

        if cmd_str in self.cmd_table.keys():
            self.vprint('running command: %s'%(cmd_str,))
            self.cmd_table[cmd_str]()
        else:
            print 'E: command, %s, not found'%(cmd_str,)
        return
            
    def cmd_no_args(self):
        cmd_str = self.args[0]
        self.motor_comm.cmd2motor_ctl(cmd_str)
        return

    def start(self):
        self.motor_comm.start()
        self.ifwait()
    
    def stop(self):
        self.motor_comm.stop()
        return

    def set_trig_ind(self):
        # Get addition command line arguments
        try:
            trig_num = self.args[1]
        except:
            print 'E: trigger numbers missing'
            sys.exit(1)
        try:
            trig_ind = self.args[2]
        except:
            print 'E: trigger indices missing'
            sys.exit(1)

        # Convert arguments to integer lists
        try:
            trig_num = [int(x) for x in trig_num.split()]
        except:
            print 'E: unable to recognize trigger numbers [%s]'%(trig_num,)
            sys.exit(1)
        try:
            trig_ind = [int(x) for x in trig_ind.split()]
        except:
            print 'E: unable to recognize trigger indices [%s]'%(trig_ind,)
            sys.exit(1)

        self.vprint('trig_num: %s'%(str(trig_num),))
        self.vprint('trig_ind: %s'%(str(trig_ind),))

        if not len(trig_num)==len(trig_ind):
            print 'E: number of triggers must match number of indices'
            sys.exit(1)
            
        # Send trigger indices to motor_ctl
        self.motor_comm.set_trig_ind(trig_num,trig_ind)

            
    def set_trig_wid(self):
        # Get addition command line arguments
        try:
            trig_num = self.args[1]
        except:
            print 'E: trigger numbers missing'
            sys.exit(1)
        try:
            trig_wid = self.args[2]
        except:
            print 'E: trigger widths missing'
            sys.exit(1)

        # Convert arguments to integer lists
        try:
            trig_num = [int(x) for x in trig_num.split()]
        except:
            print 'E: unable to recognize trigger numbers [%s]'%(trig_num,)
            sys.exit(1)
        try:
            trig_wid = [int(x) for x in trig_wid.split()]
        except:
            print 'E: unable to recognize trigger widths [%s]'%(trig_wid,)
            sys.exit(1)

        self.vprint('trig_num: %s'%(str(trig_num),))
        self.vprint('trig_wid: %s'%(str(trig_wid),))
            
        if not len(trig_num)==len(trig_wid):
            print 'E: number of triggers must match number of widths'
            sys.exit(1)
        
        # Send trigger indices to motor_ctl
        self.motor_comm.set_trig_wid(trig_num, trig_wid)

    def read_buffer(self):
        self.vprint('reading buffer')
        os_buff = self.motor_comm.read_os_buffer()
        self.vprint('writing buffer to stdout')
        if os_buffer==None:
           return
        else:
            print_buffer(os_buff)
        
    def load_buffer(self):
        try:
            filename = self.args[1]
        except:
            print 'E: missing kinematics file'
            sys.exit(1)

        self.vprint('reading file: %s'%(filename,))

        try:
            buff_data = load(filename)
            pass
        except:
            print 'E: unable to read file %s'%(filename,)
            sys.exit(1)

        self.vprint('loading buffer')
            
        try:
            self.motor_comm.load_os_buffer(buff_data)
        except ValueError, err:
            print 'E: load buffer error', err
            sys.exit(1)
        except:
            print 'E: load buffer error'
            sys.exit(1)
            
    def moveby(self):
        try:
            num_motor = self.motor_comm.num_motor()
            move = scipy.zeros((num_motor,))
            for i in range(0,num_motor):
                m = self.args[i+1]
                if m[0]=='n':
                    move[i] = -float(m[1:])
                elif m[0]=='p':
                    move[i] = float(m[1:])
                else:
                    move[i] = float(m)
        except:
            print 'E: incorrect input arguments for command moveby'
            sys.exit(1)

        self.vprint('move: %s'%(str(move),))
        self.motor_comm.moveby(move)
        self.ifwait()
                
    def move2zero(self):
        self.motor_comm.move2zero()
        self.ifwait()

    def move2pos(self):
        try:
            num_motor = self.motor_comm.num_motor()
            pos = scipy.zeros((num_motor,))
            for i in range(0,num_motor):
                p = self.args[i+1]
                if p[0]=='n':
                    pos[i] = -float(p[1:])
                elif p[0]=='p':
                    pos[i] = float(p[1:])
                else:
                    pos[i] = float(p)
        except:
            print 'E: incorrect input arguments for command move2pos'
            sys.exit(1)
        self.vprint('pos: %s'%(str(pos),))
        self.motor_comm.move2pos(pos)
        self.ifwait()

    def move2start(self):
        self.vprint('moving to start of outscan buffer')
        self.motor_comm.move2start()
        self.ifwait()
      
    def set_zero(self):
        self.motor_comm.zero_motor_ind()

    def status(self):
        self.motor_comm.print_status()
        
    def help_cmd(self):
        try:
            cmd = self.args[1]
        except:
            print 'E: need to specify which command you would like help with'
            sys.exit(1)

        try:
            print self.help_table[cmd]
        except:
            print 'E: help-cmd does not recognize command %s'%(cmd,)
        

    def help(self):
        self.parser.print_help()

    def vprint(self,msg):
        if self.options.verbose==True:
            print msg

    def ifwait(self):
        if self.options.wait==True:
            self.vprint('waiting')
            self.motor_comm.wait()

def motor_comm_main():
    cmd_line = Motor_Comm_Cmd_Line()
    cmd_line.run_command()

def load(filename):
    """
    Simple function for reading data from file 
    """
    fid = open(filename,'r')
    data = [ map(float, line.split()) for line in  fid.readlines() ]
    fid.close()
    return scipy.array(data)

def print_buffer(buff):
    n,m = buff.shape
    for i in range(0,n):
        for j in range(0,m):
            print '%d'%(buff[i,j],),
        print
    return
        
# Testing ---------------------------------------------------------
if __name__=='__main__':

    if 1:
        motor_comm_main()

    if 0:
        import pylab
        ctl = Motor_Comm()
        data = ctl.read_buffer()
        n,m=data.shape
        print data.shape
        print ctl.read_buffer('1st line')
        for i in range(0,m):
            pylab.plot(data[:,i])
        pylab.show()

    
    
