# motor_shm.py
#
# Python ctypes wrapper around libmotor_shm library. Used for
# communication with the motor_ctl hard real-time (RTAI) driver.
#
# William Dickson 01/28/2008
# -----------------------------------------------------------------
import ctypes
import scipy
import sys

DEBUG=True

def debug_print(msg):
    if DEBUG==True:
        print '\t    *%s'%(msg,)
        sys.stdout.flush()

# Load library and extract functions
libmotor_shm = ctypes.cdll.LoadLibrary("libmotor_shm.so.1")

has_trigger = libmotor_shm.has_trigger
has_pwm = libmotor_shm.has_pwm
has_clkdir = libmotor_shm.has_clkdir

# Command ID functions
id_cmd_fifo = libmotor_shm.id_cmd_fifo
id_cmd_stop = libmotor_shm.id_cmd_stop
id_cmd_start = libmotor_shm.id_cmd_start
id_cmd_standby_on = libmotor_shm.id_cmd_standby_on
id_cmd_standby_off = libmotor_shm.id_cmd_standby_off
id_cmd_enable = libmotor_shm.id_cmd_enable
id_cmd_disable = libmotor_shm.id_cmd_disable
id_cmd_unlock_buffer = libmotor_shm.id_cmd_unlock_buffer
id_cmd_lock_buffer = libmotor_shm.id_cmd_lock_buffer
id_cmd_zero_buffer_pos = libmotor_shm.id_cmd_zero_buffer_pos
id_cmd_zero_motor_ind = libmotor_shm.id_cmd_zero_motor_ind
id_cmd_loopmode_on = libmotor_shm.id_cmd_loopmode_on
id_cmd_loopmode_off = libmotor_shm.id_cmd_loopmode_off
id_cmd_set_os_buffer = libmotor_shm.id_cmd_set_os_buffer
id_cmd_set_mv_buffer = libmotor_shm.id_cmd_set_mv_buffer

cmd_fifo = '/dev/rtf%d'%(id_cmd_fifo(),)

# Commands included only if trigger exists
if has_trigger():
    id_cmd_set_trig_index = libmotor_shm.id_cmd_set_trig_index
    id_cmd_set_trig_width = libmotor_shm.id_cmd_set_trig_width
    num_trigger = libmotor_shm.num_trigger
    dflt_trigger_width = libmotor_shm.dflt_trigger_width

# Commands included only if pwm exists
if has_pwm():
    num_pwm = libmotor_shm.num_pwm
    pwm_min_pulse_ns = libmotor_shm.pwm_min_pulse_ns
    pwm_max_pulse_ns = libmotor_shm.pwm_min_pulse_ns
    pwm_period_ns = libmotor_shm.pwm_period_ns
    pwm_num_index = libmotor_shm.pwm_num_index
    pwm_ns_per_index = libmotor_shm.pwm_ns_per_index
    
# Functions which return other constants
num_stepper = libmotor_shm.num_stepper
num_motor = libmotor_shm.num_motor
success = libmotor_shm.success
error_locked = libmotor_shm.error_locked
error_malloc = libmotor_shm.error_malloc
error_size = libmotor_shm.error_size
os_period_ns = libmotor_shm.os_period_ns
os_period_s = libmotor_shm.os_period_s
os_period_s.restype = ctypes.c_double
os_frequency = libmotor_shm.os_frequency
os_frequency.restype = ctypes.c_double
index_per_rev = libmotor_shm.index_per_rev
step2deg = libmotor_shm.step2deg
step2deg.restype = ctypes.c_double
deg2step = libmotor_shm.deg2step
deg2step.restype = ctypes.c_double
buffer_max_len = libmotor_shm.buffer_max_len
mv_buffer = libmotor_shm.mv_buffer
os_buffer = libmotor_shm.os_buffer
ain_buffer = libmotor_shm.ain_buffer
num_ain = libmotor_shm.num_ain

# Functions with arguments
libmotor_shm.get_buffer_len.restype = ctypes.c_int
libmotor_shm.get_buffer_len.argstype = [
    ctypes.c_int,
]

# Set argument and return types for functions
libmotor_shm.load_buffer.restype = ctypes.c_int
libmotor_shm.load_buffer.argstype = [
    ctypes.c_int,
    ctypes.c_void_p, 
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int,
    ]
libmotor_shm.read_os_buffer.restype = ctypes.c_int
libmotor_shm.read_os_buffer.argstype = [
    ctypes.c_void_p, 
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int,
    ]
libmotor_shm.read_os_buffer_1st.restype = ctypes.c_int
libmotor_shm.read_os_buffer_1st.argstype = [
    ctypes.POINTER(ctypes.c_char),
    ctypes.c_int,
    ctypes.c_int,
    ]

libmotor_shm.shm_alloc.restype = ctypes.c_int
libmotor_shm.shm_free.restype = ctypes.c_int
shm_alloc = libmotor_shm.shm_alloc
shm_free = libmotor_shm.shm_free

def get_motor_type(type_num):
    if type_num == libmotor_shm.stepper_motor():
        return 'step'
    elif type_num == libmotor_shm.pwm_motor():
        return 'pwm'
    elif type_num == libmotor_shm.clkdir_motor():
        return 'clkdir'
    else:
        raise ValueError, 'unknown motor type'
    
class status_info_cstr(ctypes.Structure):
    _fields_ = [
        ('frequency', ctypes.c_double),
        ('outscan', ctypes.c_int),
        ('standby', ctypes.c_int),
        ('enable', ctypes.c_int),
        ('loop_mode', ctypes.c_int),
        ('buffer_max_len', ctypes.c_int),
        ('os_buffer_len', ctypes.c_int),
        ('os_buffer_start', ctypes.c_int*num_motor()),
        ('mv_buffer_len', ctypes.c_int),
        ('buffer', ctypes.c_int),
        ('buffer_cur_pos', ctypes.c_int),
        ('buffer_lock', ctypes.c_int),
        ('motor_ind', ctypes.c_int*num_motor()),
        ('motor_type', ctypes.c_int*num_motor()),
        ('debug', ctypes.c_int), 
        ('ain_data',ctypes.c_int*num_ain()),
        ('ain_buffer_len',ctypes.c_int),
        ]
    if has_trigger()==1:
        trig_fields = [
            ('trig_index', ctypes.c_int*num_trigger()),
            ('trig_width', ctypes.c_int*num_trigger()),
            ]
        _fields_.extend(trig_fields)
    if has_pwm()==1:
        pwm_fields = [
            ('pwm_zero_ns', ctypes.c_int*num_pwm()),
            ]
        _fields_.extend(pwm_fields)

    # Debug ---------------------------------------------------
    #debug = [
    #        ('debug', ctypes.c_int),
    #        ]
    #_fields_.extend(debug)
    # ---------------------------------------------------------

def get_status_info():
    debug_print('get_status_info')
    status_info = status_info_cstr()
    p_status_info = ctypes.pointer(status_info)
    debug_print('in of libmotor_shm.get_status_info(p_status_info)')   # temporary
    flag = libmotor_shm.get_status_info(p_status_info)
    debug_print ('out of libmotor_shm.get_status_info(p_status_info)') # temporary
    if flag == error_malloc():
        raise MemoryError, 'failed to allocate shared memory buffer'
    status_info_dict = {}
    for f in status_info._fields_:
        f0 = f[0]
        f0_val = status_info.__getattribute__(f[0])
        if type(f0_val) in (float, int):
            status_info_dict[f0] = f0_val
        else: 
            status_info_dict[f0] = list(f0_val)
            if f0 == 'motor_type':
                status_info_dict[f0] = [get_motor_type(n) for n in status_info_dict[f0]]
    return status_info_dict
        
def load_buffer(buff, os_data):
    debug_print('load_buffer')
    n,m = os_data.shape
    if not m==num_motor():
        raise ValueError, 'oscan data of incorrect shape'
    os_data_ptr = os_data.ctypes.data_as(ctypes.c_void_p)
    s0 = os_data.ctypes.strides[0]
    s1 = os_data.ctypes.strides[1]
    nrow = os_data.ctypes.shape[0]
    ncol = os_data.ctypes.shape[1]
    if buff == 'os_buffer':
        buff_int = os_buffer()
    elif buff == 'mv_buffer':
        buff_int = mv_buffer()
    else:
        raise ValueError, 'uknown buffer'
    flag = libmotor_shm.load_buffer(buff_int,os_data_ptr,nrow,ncol,s0,s1)
    if not flag==0:
        raise MemoryError, 'failed to load outscan buffer'

def read_ain_buffer():
    """
    Read contents of analog input buffer
    """
    debug_print('read_ain_buffer')
    buff_type = ain_buffer()
    n = libmotor_shm.get_buffer_len(buff_type)
    m = num_ain()
    if n==0:
        return None
    ain_data = scipy.zeros((n,m), scipy.int_)
    ain_data_ptr = ain_data.ctypes.data_as(ctypes.c_void_p)
    s0 = ain_data.ctypes.strides[0]
    s1 = ain_data.ctypes.strides[1]
    nrow = ain_data.ctypes.shape[0]
    ncol = ain_data.ctypes.shape[1]
    flag = libmotor_shm.read_ain_buffer(ain_data_ptr,nrow,ncol,s0,s1)
    if not flag==0:
        raise MemoryError, 'failed to read outscan buffer'
    return ain_data


def read_os_buffer(mode='full'):
    """
    Read contents of outscan buffer.

    Keywords:
      mode = '1st line' or 'full'
    """
    debug_print('read_os_buffer')
    buff_type = os_buffer()
    n = libmotor_shm.get_buffer_len(buff_type)
    m = num_motor()
    if n==0: # If buffer is empty return None
        return None
    if mode=='full':
        os_data = scipy.zeros((n,m), scipy.int_)
        os_data_ptr = os_data.ctypes.data_as(ctypes.c_void_p)
        s0 = os_data.ctypes.strides[0]
        s1 = os_data.ctypes.strides[1]
        nrow = os_data.ctypes.shape[0]
        ncol = os_data.ctypes.shape[1]
        flag = libmotor_shm.read_os_buffer(os_data_ptr,nrow,ncol,s0,s1)
        if not flag==0:
            raise MemoryError, 'failed to read outscan buffer'
    elif mode=='1st line':
        os_data = scipy.zeros((m,), scipy.int_)
        os_data_ptr = os_data.ctypes.data_as(ctypes.c_void_p)
        s = os_data.ctypes.strides[0]
        n = os_data.ctypes.shape[0]
        flag = libmotor_shm.read_os_buffer_1st(os_data_ptr,n,s)
        if not flag==0:
            raise MemoryError, 'failed to read outscan buffer'
    else:
        raise ValueError, 'unkown mode %s'%(mode,)
    return os_data
        
cmd_dict = {
    'outscan-off' : (id_cmd_stop(), 0),
    'outscan-on' : (id_cmd_start(), 0),
    'standby-on' : (id_cmd_standby_on(), 0),
    'standby-off': (id_cmd_standby_off(), 0),
    'enable' : (id_cmd_enable(), 0),
    'disable' : (id_cmd_disable(), 0),
    'unlock-buffer': (id_cmd_unlock_buffer(), 0),
    'lock-buffer': (id_cmd_lock_buffer(), 0),
    'zero-buffer-pos':(id_cmd_zero_buffer_pos(), 0),
    'zero-motor-ind': (id_cmd_zero_motor_ind(), 0),
    'loop-mode-on': (id_cmd_loopmode_on(), 0),
    'loop-mode-off': (id_cmd_loopmode_off(), 0), 
    'set-os-buffer': (id_cmd_set_os_buffer(), 0),
    'set-mv-buffer': (id_cmd_set_mv_buffer(),0)
    }

if has_trigger():
    cmd_trig = {
    'set-trig-ind': (id_cmd_set_trig_index(), 2),
    'set-trig-wid': (id_cmd_set_trig_width(), 2),
    }
    cmd_dict.update(cmd_trig)
    
# Testing ---------------------------------------------------------------------
if __name__ == '__main__':

    
    try:

        shm_alloc()

        if 0:
            status_info = get_status_info()
            print status_info

        if 0:
            
            import pylab
            os_buff = read_os_buffer()
            os_buff_1st = read_os_buffer(mode='1st line')
            n,m = os_buff.shape

        
            for i in range(0,m):
                pylab.plot(os_buff[:,i])
                pylab.show()

        if 0:
            import time
        
            for i in range(0,10):
                os_buff = read_os_buffer()
                print os_buff[0,:]
                a = scipy.ones(os_buff.shape)
                b = os_buff + a
                time.sleep(0.5)
                os_buff_1st = read_os_buffer(mode='1st line')
                print os_buff_1st
                time.sleep(0.5)

        if 1:

            os_buff = read_os_buffer()
            ain_buff = read_ain_buffer()
            print 'os_buffer: ', os_buff.shape
            for i in range(0,10):
                print os_buff[i,:]

            print 

            print 'ain_buffer: ', ain_buff.shape
            for i in range(0,10):
                print ain_buff[i,:]
            

    finally:
        print 'closing '
        shm_free()
            
