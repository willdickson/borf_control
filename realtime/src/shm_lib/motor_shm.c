/*-------------------------------------------------------------------
  motor_shm.c 
  
  Shared memory library for communication with motor_ctl realtime
  stepper/pwm motor driver. 

  Author: Will Dicksom
--------------------------------------------------------------------*/
#include <stdio.h>
#include <rtai_comedi.h>
#include "motor_ctl.h"

//#define DEBUG
#define SUCCESS 0
#define FAIL -1
#define ERROR_LOCKED -2
#define ERROR_MALLOC -3
#define ERROR_SIZE -4

// Shared memory buffers
static struct buffer_str *os_buffer_shm;
static struct buffer_str *mv_buffer_shm;
static struct status_info_str *status_info_shm;
static struct ain_buffer_str *ain_buffer_shm;

int shm_alloc(void);
int shm_free(void);
int get_buffer_len(int buff_type);
int read_os_buffer(void *data, int nrow, int ncol, int s0, int s1); 
int read_os_buffer_1st(void *data, int n, int s);
int get_status_info(struct status_info_str *status_info);
int load_buffer(int buff_type, void *data, int nrow, int ncol, int s0, int s1);

// Fucntions for accessing command IDs and other constants
int id_cmd_fifo()
{return FIFO_COMMAND;}

int id_cmd_stop()
{return CMD_STOP;}

int id_cmd_start()
{return CMD_START;}

int id_cmd_standby_on()
{return CMD_STANDBY_ON;}

int id_cmd_standby_off()
{return CMD_STANDBY_OFF;}

int id_cmd_enable()
{return CMD_ENABLE;}

int id_cmd_disable()
{return CMD_DISABLE;}

int id_cmd_unlock_buffer()
{return CMD_UNLOCK_BUFFER;}

int id_cmd_lock_buffer()
{return CMD_LOCK_BUFFER;}

int id_cmd_zero_buffer_pos()
{return CMD_ZERO_BUFFER_POS;}

int id_cmd_zero_motor_ind()
{return CMD_ZERO_MOTOR_IND; }

int id_cmd_loopmode_on()
{return CMD_LOOPMODE_ON;}

int id_cmd_loopmode_off()
{return CMD_LOOPMODE_OFF;}

#ifdef TRIG
int id_cmd_set_trig_index()
{return CMD_SET_TRIG_INDEX;}

int id_cmd_set_trig_width()
{return CMD_SET_TRIG_WIDTH;}
#endif

int id_cmd_set_os_buffer()
{return CMD_SET_OS_BUFFER;}

int id_cmd_set_mv_buffer()
{return CMD_SET_MV_BUFFER;}

int stepper_motor()
{return STEPPER_MOTOR;}

int pwm_motor()
{return PWM_MOTOR;}

int clkdir_motor()
{return CLKDIR_MOTOR;}

int num_stepper()
{return NUM_STEPPER;}

int num_motor()
{return NUM_MOTOR;}

int success()
{return SUCCESS;}

int error_locked()
{return ERROR_LOCKED;}

int error_malloc()
{return ERROR_MALLOC;}

int error_size()
{return ERROR_SIZE;}

int os_period_ns()
{return PERIOD_NS;}

double os_period_s()
{return ((float)PERIOD_NS)*1.0e-9;}

double os_frequency()
{return 1.0/os_period_s();}

int index_per_rev()
{return IND_PER_REV;}

double step2deg()
{return STEP2DEG;}

double deg2step()
{return 1.0/((double)STEP2DEG);}

int buffer_max_len()
{return BUFFER_MAX_LEN;}

int os_buffer()
{return OS_BUFFER;}

int mv_buffer()
{return MV_BUFFER;}

int ain_buffer()
{return AIN_BUFFER;}

int num_ain()
{return NUM_AIN;}

#ifdef TRIG
int num_trigger()
{return NUM_TRIG;}
#endif

#ifdef TRIG
int dflt_trigger_width()
{return DFLT_TRIG_WIDTH;}
#endif

int has_trigger()
{
#ifdef TRIG
  return 1;
#else
  return 0;
#endif
}

int has_pwm()
{
#ifdef PWM
  return 1;
#else
  return 0;
#endif
}

int has_clkdir()
{
#ifdef CLKDIR
  return 1;
#else
  return 0;
#endif
}

#ifdef PWM
int num_pwm()
{return NUM_PWM;}

int pwm_min_pulse_ns()
{return PWM_MIN_PULSE_NS;}

int pwm_max_pulse_ns()
{return PWM_MAX_PULSE_NS;}

int pwm_period_ns()
{return PWM_PERIOD_NS;}

int pwm_num_index()
{return PWM_NUM_INDEX;}

int pwm_ns_per_index()
{return PWM_NS_PER_INDEX;}
#endif

// -----------------------------------------------------------------------
// shm_alloc - allocates shared memory buffers w/ motor_ctl driver. 
//
// -----------------------------------------------------------------------
int shm_alloc(void) {

#ifdef DEBUG
  printf("\t    *shm_alloc\n");
#endif
  
  // Open shared memory 
  os_buffer_shm = (struct buffer_str *) rt_shm_alloc(
						     nam2num(SHM_OS_BUFFER),
						     sizeof(struct buffer_str),
						     USE_VMALLOC      					     
						     );
  if (os_buffer_shm==0) {
    printf("*** warning allocation of os_buffer_shm failed\n");
    return ERROR_MALLOC;
  }
  mv_buffer_shm = (struct buffer_str *) rt_shm_alloc(
						     nam2num(SHM_MV_BUFFER),
						     sizeof(struct buffer_str),
						     USE_VMALLOC
						     );
  if (os_buffer_shm==0) {
    printf("*** warning allocation of mv_buffer_shm failed\n");
    return  ERROR_MALLOC;
  }
  status_info_shm = (struct status_info_str *) rt_shm_alloc(
							    nam2num(SHM_STATUS),
							    sizeof(struct status_info_str),
							    USE_VMALLOC
							    );
  if (status_info_shm==0) {
    printf("*** warning allocation of status_info_shm failed\n");
    return ERROR_MALLOC;
  }

  ain_buffer_shm = (struct ain_buffer_str *) rt_shm_alloc(
						      nam2num(SHM_AIN_BUFFER),
						      sizeof(struct ain_buffer_str),
						      USE_VMALLOC
						      );
  if (ain_buffer_shm==0) {
    printf("*** warning allocation of os_buffer_shm failed\n");
    return ERROR_MALLOC;
  }

  return SUCCESS;
}

// ----------------------------------------------------------------------------
// shm_free - free shared memory buffers 
//
// ----------------------------------------------------------------------------
int shm_free(void) {

  int rval0;
  int rval1;
  int rval2;
  int rval3;

#ifdef DEBUG
  printf("\t    *shm_free\n");
#endif

  rval0 = rt_shm_free(nam2num(SHM_OS_BUFFER));
  if (rval0==0) {
    printf("*** warning freeing of shared memory os_buffer_shm failed\n");
  }
  rval1 = rt_shm_free(nam2num(SHM_MV_BUFFER));
  if (rval1==0) {
    printf("*** warning freeing of shared memory mv_buffer_shm failed\n");
  }
  rval2 = rt_shm_free(nam2num(SHM_STATUS));
  if (rval1==0) {
    printf("*** warning freeing of shared memory status_info_shm failed\n");
  }
  rval3 = rt_shm_free(nam2num(SHM_AIN_BUFFER));
  if (rval1==0) {
    printf("*** warning freeing of shared memory ain_buffer_shm failed\n");
  }
  return SUCCESS;
}

// --------------------------------------------------------------------------------
// get_os_buffer_len - returns length of outscan buffer. Note, that must shm_alloc 
// must be called before this function will work.
//
// Argument:
// buff_type = buffer type (OS_BUFFER or MV_BUFFER)
//
// --------------------------------------------------------------------------------
int get_buffer_len(int buff_type) {
  int len;
  struct status_info_str status_info;

#ifdef DEBUG
  printf("\t    *get_buffer_len\n");
#endif

  // Check outscan status
  get_status_info(&status_info);
  
  if (buff_type==OS_BUFFER) {
    len = os_buffer_shm->len;
  }
  else if (buff_type==MV_BUFFER) {
    len = mv_buffer_shm->len;
  }
  else if (buff_type==AIN_BUFFER) {
    len = ain_buffer_shm->len;
  }
  else {
    printf("****warning unrecognized buffer type");
    return FAIL;
  }
  return len;
}

// -------------------------------------------------------------------------------
// read_ain_buffer - reads analog input buffer
//
// Arguments: 
//
// data = pointer to numpy array data
// nrow = number of rows in array
// ncol = number of columns in array
// s0 = array strides for 0th dimension
// s1 = array stgrides for 1st dimension
// -------------------------------------------------------------------------------
int read_ain_buffer(void *data, int nrow, int ncol, int s0, int s1)
{
  int i,j;
  int *ptr;

#ifdef DEBUG
  printf("\t    *read_ain_buffer\n");
#endif

  // Check length - must be equal to os_buffer_shm->len as this is how many samples
  // we have in the ain_buffer
  if (nrow > (os_buffer_shm->len)) {
    return ERROR_SIZE;
  }
  if (ncol != NUM_AIN) {
    return ERROR_SIZE;
  }
  // Read buffer contents
  for(i=0; i<nrow; i++) {
    for(j=0; j<ncol;j++) {
      ptr = (int *)(data + i*s0 + j*s1);
      *ptr = ain_buffer_shm->data[i][j];     
    }
  }
  return SUCCESS;

}

// --------------------------------------------------------------------------------
// reads_os_buffer - reads contents of outscan buffer. Note that shared memory must 
// be allocted (using shm_alloc) must be before calling this function. Output is 
// placed in the number array  pointed to by the pointer data. 
//
// Arguments: 
//
// data = pointer to numpy array data
// nrow = number of rows in array
// ncol = number of columns in array
// s0 = array strides for 0th dimension
// s1 = array stgrides for 1st dimension

// --------------------------------------------------------------------------------
int read_os_buffer(void *data, int nrow, int ncol, int s0, int s1) 
{
  int i,j;
  int *ptr;

#ifdef DEBUG
  printf("\t    *read_os_buffer\n");
#endif

  // Check length
  if (nrow > (os_buffer_shm->len)) {
    return ERROR_SIZE;
  }
  if (ncol != NUM_MOTOR) {
    return ERROR_SIZE;
  }
  // Read buffer contents
  for(i=0; i<nrow; i++) {
    for(j=0; j<ncol;j++) {
      ptr = (int *)(data + i*s0 + j*s1);
      *ptr = os_buffer_shm->data[i][j];     
    }
  }
  return SUCCESS;
}


//---------------------------------------------------------------------------------- 
// read_os_buffer_1st - reads 1st element in outscan buffer. Note, a
// call to shm_alloc must be made before calling this function in order to allocate 
// the shared memory.
//
// Arguments:
//
// data = a pointer to numpy array data
// nrow = number of rows in array
// ncol = numbetr of columns in array
// s = the stride of the array 
// ---------------------------------------------------------------------------------
int read_os_buffer_1st(void *data, int n, int s)
{
  int i;
  int *ptr;
  int temp;

#ifdef DEBUG
  printf("\t    *read_os_buffer_1st\n");
#endif

  if (n!=NUM_MOTOR) {
    return ERROR_SIZE;
  }
  
  for(i=0;i<n;i++){
    ptr = (int *)(data + i*s); 
    *ptr= os_buffer_shm->data[0][i];     
  }
  return SUCCESS;
}

// --------------------------------------------------------------------------
// load_buffer - loads array into buffer -either os_buffer or mv_buffer. 
// Note, must call shm_alloc before this function can be used.
//
// Arguments:
//
// buff_type = buffer type (OS_BUFFER or MV_BUFFER)
// data = pointer to numpy array data
// nrow = number of rows in array
// ncol = number of columns in array
// s0 = array strides for 0th dimension
// s1 = array stgrides for 1st dimension
//
// ---------------------------------------------------------------------------
int load_buffer(int buff_type, void *data, int nrow, int ncol, int s0, int s1)
{
  int i,j;
  int *ptr;
  struct status_info_str status_info;
  struct buffer_str *buffer_shm;

#ifdef DEBUG
  printf("\t    *load_buffer\n");
#endif

  // Check outscan status
  get_status_info(&status_info);

  if (status_info.buffer_lock==ON) {
    return ERROR_LOCKED;
  }
  // Select buffer
  if (buff_type==OS_BUFFER) {
    buffer_shm = os_buffer_shm;
  }
  else if (buff_type==MV_BUFFER) {
    buffer_shm = mv_buffer_shm;
  }
  else {
    return FAIL;
  }
  // Check size
  if (nrow > BUFFER_MAX_LEN) {
    return ERROR_SIZE;
  } 
  if (ncol != NUM_MOTOR) {
    return ERROR_SIZE;
  }
  // Copy data into shared memory 
  for (i=0; i<nrow; i++) {
    for (j=0; j< ncol; j++) {
      ptr = (int *)(data + i*s0 + j*s1);
      (buffer_shm->data[i][j])=*ptr;     
    }
  }
  buffer_shm->len = nrow;

  // If we are loading a new ouscan buffer reset the ain buffer length to 0
  if (buff_type==OS_BUFFER) {
    ain_buffer_shm->len = 0;
  }
  return SUCCESS;
}

// -----------------------------------------------------------------------------
// get_status_info - read status information from motor_ctl driver. Note, that
// shared memory must be allocated (using shm_alloc) before this function will 
// work.
//
// Argument:
// status_info = pointer to status information structure.
// ----------------------------------------------------------------------------- 
int get_status_info(struct status_info_str *status_info) 
{
#ifdef DEBUG
  printf("\t    *get_status_info\n");
#endif

  // Copy status info 
  *status_info = *status_info_shm;

  return SUCCESS;
}




