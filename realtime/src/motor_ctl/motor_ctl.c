//
// motor_ctl.c - RTAI based hard realtime motor control driver
//
// Author: Will Dickson 
//
// ----------------------------------------------------------------------
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <rtai.h>
#include <rtai_sched.h>
#include <rtai_spl.h>
#include <rtai_fifos.h>
#include <rtai_wd.h>
#include <comedilib.h>
#include "motor_ctl.h"

MODULE_DESCRIPTION("Real-time stepper/pwm motor driver w/triggering");
MODULE_AUTHOR("William Dickson (wbd@caltech.edu)");
MODULE_LICENSE("None");

#define DIO_LO 0
#define DIO_HI 1

// Parallel Port addresses 
#define DATAPORT 0x378
#define STATPORT 0x379
#define CTRLPORT 0x37A
#define RWL_LOCK_DELAY 10000
#define NUM_STEP 8

// Masks various control/outputs  
#define MASK_ENABLE  0x001
#define MASK_DISABLE 0x00e
#define MASK_STANDBY_ON 0x00b
#define MASK_STANDBY_OFF 0x004

// Realtime task state structure
struct sys_state_str
{
  RT_TASK main_task;
  volatile int outscan;
  volatile int standby;
  volatile int enable;
  volatile int buffer_lock;
  volatile int buffer_pos;
  volatile int motor_index[NUM_MOTOR];
  volatile int motor_type[NUM_MOTOR];
  volatile int loop_mode;
  volatile int buffer;
  comedi_t *device;
  volatile lsampl_t ain_data[NUM_AIN];
};

// Trigger state structure
#ifdef TRIG
struct trig_state_str
{
  volatile int index[NUM_TRIG];
  volatile int status[NUM_TRIG];
  volatile int count[NUM_TRIG];
  volatile int width[NUM_TRIG];
};
#endif

// Function proto types 
static int __motor_ctl_init(void);
static void __motor_ctl_exit(void);
int cmd_handler(unsigned int fifo, int rw);
void main_task_func(int long);
void init_sys_state(void);
void set_status_info(void);
void send_motor_cmds(void);
void acquire_data(void);
void set_clk_low(void);
#ifdef TRIG
void init_trig_state(void);
void init_trig_dio_pins(void); // new trig function
void send_triggers(void);
#endif

// Motor half steps 
const char motor_step[NUM_STEP][NUM_STEPPER] ={
  {0x0e,0xe0},
  {0x0c,0xc0},
  {0x0d,0xd0},
  {0x09,0x90}, 
  {0x0b,0xb0},
  {0x03,0x30},
  {0x07,0x70},
  {0x06,0x60}
};

// Trigger output pins
#ifdef TRIG
static int trig_dio_pins[NUM_TRIG] = TRIG_DIO_PINS;
#endif

// Clock and direction output pins
static int clk_dio_pins[NUM_CLKDIR] = CLK_DIO_PINS;
static int dir_dio_pins[NUM_CLKDIR] = DIR_DIO_PINS;

// Analog inputs
const int ain_chan[NUM_AIN] = AIN_CHAN;      
const int ain_aref[NUM_AIN] = AIN_AREF;

// Global state information 
static struct sys_state_str sys_state;
#ifdef TRIG
static struct trig_state_str trig_state;
#endif

// Shared memory buffers
static struct status_info_str *status_info;
static struct buffer_str *os_buffer;
static struct buffer_str *mv_buffer;
static struct buffer_str *buffer;
static struct ain_buffer_str *ain_buffer;

// ----------------------------------------------------------------------------
// main_task_func - main real-time task function
//
// ----------------------------------------------------------------------------
void main_task_func(long arg) {
  
  RTIME now_ns;

  for (;;) {
    now_ns = rt_get_time_ns();
#ifdef TRIG
      send_triggers();
#endif
    acquire_data();
    send_motor_cmds();
    set_status_info();

    // Sleep for a bit and then set the clks low
    rt_sleep_until(nano2count(rt_get_time_ns() + CLK_HI_TIME));
    set_clk_low(); 

    // Sleep until next cycle 
    now_ns += (PERIOD_NS);
    rt_sleep_until(nano2count(now_ns));    
  } 
  return;
} 

// ---------------------------------------------------------------------------
// init_sys_state - initializes system state
//
// Note, called once by hard realtime loop
// ---------------------------------------------------------------------------
void init_sys_state(void)
{
  int i;
  sys_state.outscan = OFF;          
  sys_state.standby = ON;
  sys_state.enable = OFF;
  sys_state.buffer_lock = ON;
  sys_state.buffer_pos = 0;
  sys_state.loop_mode = OFF;
  sys_state.buffer = OS_BUFFER;
  // Set motor position index and type
  for (i=0; i<NUM_MOTOR; i++) {
    sys_state.motor_index[i] = 0;
    if (i < NUM_STEPPER ) {  
      sys_state.motor_type[i] = STEPPER_MOTOR;
    }
    else {
      sys_state.motor_type[i] = CLKDIR_MOTOR;
    }
  }
  return;
}

// --------------------------------------------------------------------------
// init_trig_state - initialize triggers
//
// Note, called once by main hard realtime task
// --------------------------------------------------------------------------

#ifdef TRIG
void init_trig_state(void)
{
  int i;
  for (i=0; i<NUM_TRIG; i++){
    trig_state.index[i] = -1;
    trig_state.status[i] = OFF;
    trig_state.count[i] = 0;
    trig_state.width[i] = DFLT_TRIG_WIDTH;
  }
  return;
}
#endif

// ----------------------------------------------------------------------------
// send_triggers - sends digital IO triggers
//
// Note, part of realtime loop
// ----------------------------------------------------------------------------
#ifdef TRIG
void send_triggers(void)
{
  int i;
  int buffer_pos;
  
  if (sys_state.buffer==OS_BUFFER) {
  
    // Get buffer position
    buffer_pos = sys_state.buffer_pos;
  
    for (i=0; i<NUM_TRIG; i++) {
    
      // Turn on trigger if index is equal to current buffer position 
      if (trig_state.index[i] == buffer_pos && sys_state.outscan==ON) {
	trig_state.status[i] = ON;
      }

      // IF state is ON set HI and increment counter otherwise set LO
      if (trig_state.status[i]==ON) {
	comedi_dio_write(sys_state.device, DIO_SUBDEV, trig_dio_pins[i], DIO_HI);	
	trig_state.count[i]+= 1;
      }
      //else {
      //  comedi_dio_write(sys_state.device, DIO_SUBDEV, trig_dio_pins[i], DIO_LO);	
      //}

      // Terminate trigger at approriate width 
      if (trig_state.count[i]>=trig_state.width[i]) {
	trig_state.status[i] = OFF;
	trig_state.count[i] = 0;
	comedi_dio_write(sys_state.device, DIO_SUBDEV, trig_dio_pins[i], DIO_LO);	
      
      }
    }
  }
  return;
}
#endif

// --------------------------------------------------------------------------
// send_motor_cmds - send movement commands to motors. 
//
// Note, part of realtime loop
// ---------------------------------------------------------------------------
void send_motor_cmds(void)
{
  int i,j;
  static int motor_step_pos[NUM_STEPPER]; // inititalized to zeros because static
  static int motor_start_pos[NUM_MOTOR];  // ditto
  static int last_outscan_state = OFF;
  int sys_index_rel = 0;
  int buf_index_0 = 0;
  int buf_index_rel = 0;
  char para_out;
  static int temp[NUM_CLKDIR];
  
  if (sys_state.outscan == ON) {      

    // Loop over all motors - steppers and pwm motors
    for (i=0;i<NUM_MOTOR;i++){

      // Select buffer to outscan
      if (sys_state.buffer==OS_BUFFER) {
	buffer = os_buffer;
      }
      else {
	buffer = mv_buffer;
      }

      // Has the outscan been turned on after being off  -  buffer could have been changed
      // length. Check and set buffer pos to zero if outside of range.
      if (last_outscan_state == OFF) {
	if (sys_state.buffer_pos >= buffer->len) {
	  sys_state.buffer_pos = 0;
	}
      }
            
      // Are we srarting from the begining of the buffer ? 
      // If so get relative motor position reset the analog input buffer position 
      // if this is an outscan.
      if (sys_state.buffer_pos==0) {
	motor_start_pos[i] = sys_state.motor_index[i];
	if (sys_state.buffer==OS_BUFFER) {
	  ain_buffer->len = 0; 
	}
      }
      sys_index_rel = sys_state.motor_index[i]-motor_start_pos[i];
      buf_index_0 = buffer->data[0][i];
      buf_index_rel = buffer->data[sys_state.buffer_pos][i] - buf_index_0;

      // Forward step	
      if (buf_index_rel > sys_index_rel) {

	// -----------------------------------------------------------
	// Add of check motor indices for overrange - condition
	// -----------------------------------------------------------
	
	sys_state.motor_index[i] += 1;
	
	// For steppers update motor step position
	if (sys_state.motor_type[i]==STEPPER_MOTOR) {
	  motor_step_pos[i] = motor_step_pos[i]+1;
	  if (motor_step_pos[i] >= NUM_STEP){
	    motor_step_pos[i] = 0;
	  }
	}
	
	// For CLKDIR motors set direction to positive and clock motors
	if (sys_state.motor_type[i]==CLKDIR_MOTOR) {
	  j = i - NUM_STEPPER;
	  // set direction
	  comedi_dio_write(sys_state.device, DIO_SUBDEV, dir_dio_pins[j], DIR_POS);
	  // set clock high
	  comedi_dio_write(sys_state.device, DIO_SUBDEV, clk_dio_pins[j], DIO_HI);
	}	
      } // End foward step

      // Backward step
      else if(buf_index_rel < sys_index_rel) { 
	
	// -----------------------------------------------------------
	// Add of check motor indices for underrange - condition
	// -----------------------------------------------------------
		
	sys_state.motor_index[i] -= 1;

	// For steppers update motor step position
	if (sys_state.motor_type[i]==STEPPER_MOTOR) {
	  motor_step_pos[i] = motor_step_pos[i]-1;
	  if (motor_step_pos[i] < 0) {
	    motor_step_pos[i] = NUM_STEP-1;
	  }
	}
	
	// For CLKDIR motors set direction to negative and clock motors
	if (sys_state.motor_type[i]==CLKDIR_MOTOR) {
	  j = i-NUM_STEPPER;
	  // set direction
	  comedi_dio_write(sys_state.device, DIO_SUBDEV, dir_dio_pins[j], DIR_NEG);
	  // set clock hi
	  comedi_dio_write(sys_state.device, DIO_SUBDEV, clk_dio_pins[j], DIO_HI);
	}
	
      } // End Backward step

    } // End for i
    
    // Send commands to stepper motors. 
    para_out = 0x00;
    for (i=0; i<NUM_STEPPER; i++){
      para_out = para_out | motor_step[motor_step_pos[i]][i];
    }    
    outb(para_out,DATAPORT);

    // Put data in ain_buffer - if this is an outscan
    if (sys_state.buffer == OS_BUFFER) {
      for (i=0; i< NUM_AIN; i++) {
	ain_buffer -> data[sys_state.buffer_pos][i] = (int) sys_state.ain_data[i];
      }
      ain_buffer -> len += 1;
    }

    // Update position in outscan buffer - if at the end of buffer reset to zero 
    sys_state.buffer_pos+=1;
    if (sys_state.buffer_pos >= buffer->len) {
      sys_state.buffer_pos = 0;
      if (sys_state.loop_mode==OFF) {
	sys_state.outscan = OFF;
      }
    }

  } // end if (outscan==ON) 

  last_outscan_state = sys_state.outscan;
  
  return;
} // end send_motor_cmds 

//---------------------------------------------------------------------------
// set_clk_low - set all clock pins low
//
// Note, part of realtime loop
// --------------------------------------------------------------------------
void set_clk_low(void)
{
  int i;
  for (i=0; i<NUM_CLKDIR; i++) {
    comedi_dio_write(sys_state.device, DIO_SUBDEV, clk_dio_pins[i], DIO_LO);
  }
}


//---------------------------------------------------------------------------
// acquire_data - acquires data from analog input card
//
// Note, part of realtime loop
// --------------------------------------------------------------------------
void acquire_data(void)
{
  int i;
  
  // Acquire data
  if (sys_state.buffer==OS_BUFFER) {
    for(i=0; i<NUM_AIN; i++) {
      comedi_data_read(
		       sys_state.device, 
		       AIN_SUBDEV, ain_chan[i], 
		       AIN_RANGE, ain_aref[i], 
		       &sys_state.ain_data[i]
		       );
    }
  }
}



//---------------------------------------------------------------------------
// set_status_info - updates status info shared memory for access from 
// user space tasks.
//
// Note, part of realtime loop
// --------------------------------------------------------------------------
void set_status_info()
{
  int i;

  status_info -> freq = 1.0/((1.0e-9)*(float)PERIOD_NS);
  status_info -> outscan = sys_state.outscan;
  status_info -> standby = sys_state.standby;
  status_info -> enable = sys_state.enable;
  status_info -> loop_mode = sys_state.loop_mode;
  status_info -> buffer_max_len = BUFFER_MAX_LEN;
  status_info -> buffer_lock = sys_state.buffer_lock;
  status_info -> buffer_cur_pos = sys_state.buffer_pos;
  status_info -> buffer = sys_state.buffer;
  for (i=0; i<NUM_MOTOR; i++ ) {
    status_info -> os_buffer_start[i] = os_buffer -> data[0][i];
    status_info -> motor_ind[i] = sys_state.motor_index[i];
    status_info -> motor_type[i] = sys_state.motor_type[i];
  }
  // Analog input values
  for (i=0; i<NUM_AIN; i++) {
    status_info -> ain_data[i] = (int) sys_state.ain_data[i];
  }
  // Buffer lengths
  status_info -> os_buffer_len = os_buffer -> len;
  status_info -> mv_buffer_len = mv_buffer -> len;
  status_info -> ain_buffer_len = ain_buffer ->len;
    
#ifdef TRIG
  // Copy trigger information
  for(i=0; i<NUM_TRIG; i++) {
    status_info -> trig_index[i] = trig_state.index[i];
    status_info -> trig_width[i] = trig_state.width[i];
  }
#endif
  
  return;
}

//-------------------------------------------------------------------------
// init_trig_dio_pins - intialize bit mask for pwm output
//
// ------------------------------------------------------------------------
/*
#ifdef TRIG
void init_trig_dio_pins(void)
{
  int i; 
  for (i=0; i<NUM_TRIG; i++) {
    trig_dio_pins[i] = i;
  }
  return;
}
#endif
*/

// -------------------------------------------------------------------------
// cmd_handler - handler function for Command FIFO. This function is 
// resposible for handling commands written from user space to the command
// fifo.
//
// Note, not realtime.
// -------------------------------------------------------------------------
int cmd_handler(unsigned int fifo, int rw)
{
  int i;  
  int msg_sz;
  int cmd; 
  char port_data;
  static struct buffer_str *cur_buffer;
#ifdef TRIG
  int index;
  int data[2];
  int trig_num = 0;
  int trig_ind = 0;
  int trig_wid = 0;
#endif

  //rt_printk("motor_ctl: begin cmd_handler\n");
  
  if (rw=='r') {
    // On read - we shouldn't get any of these
  }
  else {
    // Note, rw doesn't seem to take the right value on writes ??
    msg_sz = rtf_get(FIFO_COMMAND, &cmd, sizeof(int));

    switch(cmd) {
    
    case CMD_STOP:
      sys_state.outscan = OFF;
      break;

    case CMD_START:
      if ( (sys_state.outscan==OFF) && (sys_state.buffer_lock==ON)) {
	sys_state.outscan = ON;
      }
      break;

    case CMD_UNLOCK_BUFFER:
      if (sys_state.outscan==OFF) {
	sys_state.buffer_lock = OFF;
      }
      break;

    case CMD_LOCK_BUFFER:
      sys_state.buffer_lock = ON;
      break;

    case CMD_ZERO_BUFFER_POS:
      if (sys_state.outscan==OFF) {
	sys_state.buffer_pos = 0;
      }
      break;

    case CMD_STANDBY_ON:
      port_data = inb(CTRLPORT);
      port_data = port_data & MASK_STANDBY_ON;
      outb(port_data,CTRLPORT);
      sys_state.standby = ON;
      break;

    case CMD_STANDBY_OFF:      
      port_data = inb(CTRLPORT);
      port_data = port_data | MASK_STANDBY_OFF;
      outb(port_data,CTRLPORT);
      sys_state.standby = OFF;
      break;
      
    case CMD_ENABLE:
      port_data = inb(CTRLPORT);
      port_data = port_data | MASK_ENABLE;
      outb(port_data,CTRLPORT);
      sys_state.enable = ON;
      break;

    case CMD_DISABLE:
      port_data = inb(CTRLPORT);
      port_data = port_data & MASK_DISABLE;
      outb(port_data,CTRLPORT);
      sys_state.enable = OFF;
      break;

    case CMD_ZERO_MOTOR_IND:
      if (sys_state.outscan==OFF) {
	for(i=0; i<NUM_MOTOR; i++ ) {
	  sys_state.motor_index[i] = 0;
	}
      }
      break;

    case CMD_LOOPMODE_ON:
      sys_state.loop_mode=ON;
      break;

    case CMD_LOOPMODE_OFF:
      sys_state.loop_mode=OFF;
      break;

#ifdef TRIG
    case CMD_SET_TRIG_INDEX:
      msg_sz = rtf_get(FIFO_COMMAND, &data, 2*sizeof(int)); 
      trig_num = data[0];
      trig_ind = data[1];
      if (trig_num>=0 && trig_num<NUM_TRIG) {
	trig_state.index[trig_num] = trig_ind; 
      }
      break;

    case CMD_SET_TRIG_WIDTH:
      msg_sz = rtf_get(FIFO_COMMAND, &data, 2*sizeof(int)); 
      trig_num = data[0];
      trig_wid = data[1];
      if (trig_num>=0 && trig_num<NUM_TRIG) {
	trig_state.width[trig_num] = trig_wid; 
      }
      break;
#endif

    case CMD_SET_OS_BUFFER:
      if ((sys_state.buffer_lock==OFF) &&(sys_state.outscan==OFF)) {
	sys_state.buffer_pos = 0;
	sys_state.buffer = OS_BUFFER;
      }
      break;

    case CMD_SET_MV_BUFFER:
      if ((sys_state.buffer_lock==OFF) && (sys_state.outscan==OFF)) {
	sys_state.buffer_pos = 0;
	sys_state.buffer = MV_BUFFER;
      }
      break;
    } 
  } 
  //rt_printk("motor_ctl: end cmd_handler\n");
  return 0;
}

// -----------------------------------------------------------------------
// __motor_ctl_init - kernel module initialization function 
//
//
// Need to handle initialization failure better. Upon failure need to 
// unregister all registered objects and exit with an error code.
//
// -----------------------------------------------------------------------
static int __motor_ctl_init(void)
{
  int i;
  int rval = 0;
  int rttask_param = 0;
  int rttask_stack = 3000;
  int rttask_priority =1;
  int rttask_use_fp=0;
  size_t fifo_sz;
  void *rttask_signal=0;

  // Open and configure comedi devicde
  sys_state.device = comedi_open(COMEDI_DEV);
  if (sys_state.device==0) {
    printk("motor_ctl: unable to open comedi device\n");
    return -ENODEV;
  }
  comedi_lock(sys_state.device,AIN_SUBDEV); // This is a bit brutal

  // Initialize DIO pin configuration
#ifdef TRIG
  //init_trig_dio_pins();
  for (i=0; i<NUM_TRIG; i++) {
    comedi_dio_config(sys_state.device, DIO_SUBDEV, trig_dio_pins[i], COMEDI_OUTPUT);
  } 
#endif
  for (i=0; i<NUM_CLKDIR; i++) {
    comedi_dio_config(sys_state.device, DIO_SUBDEV, clk_dio_pins[i], COMEDI_OUTPUT);
    comedi_dio_config(sys_state.device, DIO_SUBDEV, dir_dio_pins[i], COMEDI_OUTPUT);
    printk("motor_ctl: configuring clkdir pins \n");
  }
  comedi_lock(sys_state.device,DIO_SUBDEV); // This is a bit brutal
  
  // Setup command fifo and handler 
  fifo_sz = 3*sizeof(int);
  rval = rtf_create(FIFO_COMMAND, fifo_sz);
  if (rval!= 0 ) {
    printk("motor_ctl: (%d) failed to create fifo_command \n", rval);
    return -ENOMEM;    
  } 
  rval = rtf_create_handler(FIFO_COMMAND, X_FIFO_HANDLER(cmd_handler));
  if (rval!=0) {
    printk("motor_ctl: failed to create fifo handler");
    return -ENOMEM;
  }

  // Allocate shared memory for status information 
  status_info = rt_shm_alloc(nam2num(SHM_STATUS), sizeof(struct status_info_str),USE_VMALLOC);
  if (status_info == 0) {
    printk ("motor_ctl: can't allocate shared memory\n");
    return -ENOMEM;
  }

  // Allocate shared memory for outscan buffer 
  os_buffer = rt_shm_alloc(nam2num(SHM_OS_BUFFER), sizeof(struct buffer_str), USE_VMALLOC);
  if (os_buffer == 0) {
    printk ("motor_ctl: can't allocate shared memory for outscan buffer\n");
    return -ENOMEM;
  }
  
  // Allocate shared memory for outscan buffer 
  mv_buffer = rt_shm_alloc(nam2num(SHM_MV_BUFFER), sizeof(struct buffer_str), USE_VMALLOC);
  if (mv_buffer == 0) {
      printk ("motor_ctl: can't allocate shared memory for move buffer\n");
      return -ENOMEM;
  }
  
  // Allocate Data acquisition buffer  
  ain_buffer = rt_shm_alloc(nam2num(SHM_AIN_BUFFER), sizeof(struct ain_buffer_str), USE_VMALLOC);
  if (ain_buffer == 0) {
    printk ("motor_ctl: can't allocate shared memory for ain buffer\n");
    return -ENOMEM;
  }
  
  // Start timer and initialize main rt task 
  rt_set_oneshot_mode(); 
  start_rt_timer(0); 
  rt_task_init(
	       &sys_state.main_task,
	       main_task_func,
	       rttask_param,
	       rttask_stack,
	       rttask_priority,
	       rttask_use_fp,
	       rttask_signal
	       );

  // Add floating point support 
  rt_linux_use_fpu(1); 

  // Set lines to zero
  outb(0x000,CTRLPORT);
  outb(0x000,DATAPORT);
#ifdef TRIG 
  for (i=0; i<NUM_TRIG; i++){
    comedi_dio_write(sys_state.device, DIO_SUBDEV, trig_dio_pins[i], DIO_LO);	
  }
#endif

  // Initialize system state and triggers 
  init_sys_state();
#ifdef TRIG
  init_trig_state();
#endif

  // Start up the rt tasks  
  rt_task_resume(&sys_state.main_task);

  printk ("motor_ctl: started \n");
  return 0;
}


// ---------------------------------------------------------------------------
// __motor_ctl_exit - kernel module exit function.
//
// ---------------------------------------------------------------------------
static void __motor_ctl_exit(void)
{
  int rval;

  // Delete the rt tasks   
  rt_task_delete(&sys_state.main_task);

  stop_rt_timer(); 

  // Close the comedi device
  comedi_unlock(sys_state.device,AIN_SUBDEV);
  comedi_unlock(sys_state.device,DIO_SUBDEV); 
  rval = comedi_close(sys_state.device);
  if (rval==-1) {
    printk("failed to close comedi device\n");
  }
  
  // Disable motors, standyby mode, etc 
  outb(0x000,CTRLPORT);
    
  // Destroy the FIFOS 
  rtf_destroy(FIFO_COMMAND);

  // Free shared memory 
  rt_shm_free(nam2num(SHM_STATUS));
  rt_shm_free(nam2num(SHM_OS_BUFFER));
  rt_shm_free(nam2num(SHM_MV_BUFFER));
  rt_shm_free(nam2num(SHM_AIN_BUFFER));

  // OK to use printk here 
  printk ("motor_ctl: stopped \n"); 
  return; 
}

// Register module with kernel
module_init(__motor_ctl_init);
module_exit(__motor_ctl_exit);

