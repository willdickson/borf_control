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
#define AIN_AREF {AREF_GROUND,AREF_GROUND,AREF_GROUND,AREF_GROUND,AREF_GROUND,AREF_GROUND}

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
#define BUFFER_MAX_LEN 1000000
#define ON 1
#define OFF 0
#define STEPPER_MOTOR 0
#define CLKDIR_MOTOR 2
#define OS_BUFFER 0
#define MV_BUFFER 1
#define AIN_BUFFER 3

// Stepper constants
#define STEP2DEG 0.9
#define NUM_STEPPER 2
#define IND_PER_REV 400

// Clock and Direction constants
#define NUM_CLKDIR 1
#define CLK_HI_TIME (PERIOD_NS/2)
#define CLK_DIO_PINS {0}
#define DIR_DIO_PINS {1}
#define DIR_POS 1
#define DIR_NEG 0

// Trigger constants
#ifdef TRIG
#define NUM_TRIG 2 
#define DFLT_TRIG_WIDTH 20
#define TRIG_DIO_PINS {6,7}
#endif 

#define NUM_MOTOR (NUM_STEPPER+NUM_CLKDIR)

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



