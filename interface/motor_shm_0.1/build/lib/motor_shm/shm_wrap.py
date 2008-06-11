# motor_shm.py
#
# Python ctypes wrapper around libmotor_shm library. Used for
# communication with the motor_ctl hard real-time (RTAI) driver.
#
# William Dickson 01/28/2008
# -----------------------------------------------------------------
import ctypes
import scipy

# Load library and extract functions
libmotor_shm = ctypes.cdll.LoadLibrary("libmotor_shm.so.1")

has_trigger = libmotor_shm.has_trigger
has_pwm = libmotor_shm.has_pwm

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

def get_motor_type(type_num):
    if type_num == libmotor_shm.stepper_motor():
        return 'step'
    elif type_num == libmotor_shm.pwm_motor():
        return 'pwm'
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
        ('buffer_cur_len', ctypes.c_int),
        ('buffer_cur_pos', ctypes.c_int),
        ('buffer_lock', ctypes.c_int),
        ('motor_ind', ctypes.c_int*num_motor()),
        ('motor_type', ctypes.c_int*num_motor()),
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
        
def get_status_info():
    status_info = status_info_cstr()
    p_status_info = ctypes.pointer(status_info)
    flag = libmotor_shm.get_status_info(p_status_info)
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

        
def load_os_buffer(os_data):
    _os_data = scipy.ascontiguousarray(os_data)
    n,m = _os_data.shape
    if not m==num_motor():
        raise ValueError, 'oscan data of incorrect shape'
    os_data_ptr = _os_data.ctypes.data_as(ctypes.POINTER(ctypes.c_int))
    flag = libmotor_shm.load_os_buffer(os_data_ptr,n)
    if not flag==0:
        raise MemoryError, 'failed to load outscan buffer'
    
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
    'set-trig-ind': (id_cmd_set_trig_index(), 2),
    'set-trig-wid': (id_cmd_set_trig_width(), 2),
    }

# Testing ---------------------------------------------------------------------
if __name__ == '__main__':

    status_info = get_status_info()
    print status_info

