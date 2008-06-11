// -------------------------------------------------------------------------
//
// motor_ctl.h - header file for motor_ctl RTAI based motor driver and
// libshm shared memory library
//
// Author: Will Dickson 
//
// --------------------------------------------------------------------------

#include <rtai.h>
#include <rtai_shm.h>
#include <rtai_nam2num.h>
#include <rtai_rwl.h>

// Currently TRIG and PWM options cannot be used with CLKDIR options
#if defined(CLKDIR) && ( defined(PWM) || defined(TRIG))
#error CLKDIR option cannnot be used with PWM ot TRIG options 
#endif

// FIFO IDs 
#define FIFO_COMMAND 0

// Shared memory names 
#define SHM_STATUS "status"
#define SHM_OS_BUFFER "osbuff"
#define SHM_MV_BUFFER "mvbuff"
#define SHM_AIN_BUFFER "ainbuff"

// Data aquisition
#define COMEDI_DEV "/dev/comedi0"
#define AIN_RANGE 0
#define AIN_SUBDEV 0
#define NUM_AIN 6
#define AIN_CHAN {0,1,2,3,4,5}
#define AIN_AREF {AREF_DIFF,AREF_DIFF,AREF_DIFF,AREF_DIFF,AREF_DIFF,AREF_DIFF} 

// DAQ card digital IO
#define DIO_SUBDEV 2

// Command IDs 
#define CMD_STOP 0
#define CMD_START 1
#define CMD_STANDBY_ON 2
#define CMD_STANDBY_OFF 3
#define CMD_ENABLE 4
#define CMD_DISABLE 5
#define CMD_UNLOCK_BUFFER 6
#define CMD_LOCK_BUFFER 7
#define CMD_ZERO_BUFFER_POS 8
#define CMD_ZERO_MOTOR_IND 9 
#define CMD_LOOPMODE_ON 10
#define CMD_LOOPMODE_OFF 11
#define CMD_SET_TRIG_INDEX 12
#define CMD_SET_TRIG_WIDTH 13
#define CMD_SET_OS_BUFFER 14
#define CMD_SET_MV_BUFFER 15

// Constants 
#define PERIOD_NS 500000
#define BUFFER_MAX_LEN 1500000
#define ON 1
#define OFF 0
#define STEPPER_MOTOR 0
#define PWM_MOTOR 1
#define CLKDIR_MOTOR 2
#define OS_BUFFER 0
#define MV_BUFFER 1
#define AIN_BUFFER 3

// Stepper constants
#define STEP2DEG 0.9
#define NUM_STEPPER 2
#define IND_PER_REV 400

// Clock and Direction constants
#ifdef CLKDIR
#define NUM_CLKDIR 2
#define CLK_HI_TIME 50000
#define CLK_DIO_PINS {0,2}
#define DIR_DIO_PINS {1,3}
#define DIR_POS 1
#define DIR_NEG 0
#endif

// Trigger constants
#ifdef TRIG
#define NUM_TRIG 7 //Max=8
#define DFLT_TRIG_WIDTH 20
#endif 

// PWM constants
#if defined(TRIG) && defined(PWM)
#define NUM_PWM (8-NUM_TRIG)
#elif defined(PWM)
#define NUM_PWM 8
#endif
#ifdef PWM
#define PWM_MIN_PULSE_NS 900000 
#define PWM_MAX_PULSE_NS 2100000
#define PWM_EARLY_NS 10000
#define PWM_PERIOD_NS 20000000
#define PWM_NUM_INDEX 800
#define PWM_NS_PER_INDEX ((PWM_MAX_PULSE_NS-PWM_MIN_PULSE_NS )/(PWM_NUM_INDEX-1))
#endif

// Need to change code so that pwm works with CLKDIR
#if defined(PWM) && !defined(CLKDIR)
#define NUM_MOTOR (NUM_STEPPER+NUM_PWM) 
#endif

#if defined(CLKDIR) && !defined(PWM)
#define NUM_MOTOR (NUM_STEPPER+NUM_CLKDIR)
#endif

#if defined(PWM) && defined(CLKDIR)
#define NUM_MOTOR (NUM_STEPPER+NUM_CLKDIR+NUM_PWM)
#endif

#if !defined(PWM) && !defined(CLKDIR)
#define NUM_MOTOR NUM_STEPPER
#endif


// Status info struture 
struct status_info_str 
{
  double freq;
  int outscan; 
  int standby;
  int enable;
  int loop_mode;
  int buffer_max_len;
  int os_buffer_len;
  int os_buffer_start[NUM_MOTOR];
  int mv_buffer_len;
  int buffer;
  int buffer_cur_pos;
  int buffer_lock;
  int motor_ind[NUM_MOTOR];
  int motor_type[NUM_MOTOR];
  int debug;
  int ain_data[NUM_AIN];
  int ain_buffer_len;
#ifdef TRIG
  int trig_index[NUM_TRIG];
  int trig_width[NUM_TRIG];
#endif
#ifdef PWM
  int pwm_zero_ns[NUM_PWM];
#endif

};

// Outscan buffer structure 
struct buffer_str
{
  int len;
  int data[BUFFER_MAX_LEN][NUM_MOTOR];
};

// Data acquisition buffer
struct ain_buffer_str
{ int len;
  int data[BUFFER_MAX_LEN][NUM_AIN];
};



