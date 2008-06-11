#!/usr/bin/env python
import curses
import time
import scipy
import os
import ctypes 
from motor_shm import get_status_info
from motor_shm import get_motor_type
from motor_shm import num_motor
from motor_shm import has_trigger
from motor_shm import has_pwm
from motor_comm import cmd2motors
if has_trigger()==1:
    from motor_shm import num_trigger

OFF = 0
ON = 1

# List of status info fields in desired output order
status_fields = [
    'frequency',
    'outscan',
    'standby',
    'enable',
    'loop_mode',
    'buffer_max_len',
    'buffer_cur_len',
    'buffer_cur_pos',
    'buffer_lock',
    'motor_ind',
    'motor_type',
    ]

if has_pwm()==1:
    pwm_fields = ['pwm_zero_ns']
    status_fields.extend(pwm_fields)

if has_trigger()==1:
    trigger_fields = ['trig_index','trig_width']
    status_fields.extend(trigger_fields)
    
    
class motor_status:

    def __init__(self):

        self.row=1
        self.col=1
        self.step=1
        self.stdscr = curses.initscr()
        self.indent = 1
        self.header_str = 'arrick_ctl status'
        self.num_motor = num_motor()
        self.status_info = {}
        if has_trigger()==1:
            self.has_trigger = 1
            self.num_trigger = num_trigger()
        else:
            self.has_trigger = 0
        
    def run(self):
        
        try:
            self.initialize()
            done = False
            while not done:
                try:                    
                    # Display data
                    self.row, self.col=1,1
                    self.stdscr.clear()
                    self.display_header()
                    self.display_status()
                    time.sleep(0.4)
                    
                    # Get command
                    key = self.stdscr.getch()
                    if key==curses.ERR or key==curses.KEY_RESIZE:
                        continue
                    if key==ord('q'):
                        done=True
                        continue
                    try:
                        if key==ord('o'):
                            if self.status_info['outscan']==OFF:
                                 cmd2motors('outscan-on')
                            else:
                                 cmd2motors('outscan-off')
                        elif key==ord('s'):
                            if self.status_info['standby']==OFF:
                                 cmd2motors('standby-on')
                            else:
                                 cmd2motors('standby-off')
                        elif key==ord('e'):
                            if self.status_info['enable']==OFF:
                                 cmd2motors('enable')
                            else:
                                 cmd2motors('disable')
                        elif key==ord('l'):
                            if self.status_info['loop_mode']==OFF:
                                 cmd2motors('loop-mode-on')
                            else:
                                 cmd2motors('loop-mode-off')
                        elif key==ord('b'):
                            cmd2motors('zero-buffer-pos')
                        elif key==ord('m'):
                            cmd2motors('zero-motor-ind')                    
                    except KeyError:   
                        pass
                    except IOError:
                        pass
                                            
                except KeyboardInterrupt:
                    done=True

        finally:
            self.cleanup()
                   
    def initialize(self):
        curses.cbreak()
        curses.noecho()
        self.stdscr.nodelay(1)
        self.stdscr.keypad(1)
        curses.curs_set(0)       
           
    def cleanup(self):
        curses.curs_set(1)
        self.stdscr.clear()
        curses.nocbreak()
        self.stdscr.keypad(0)
        curses.echo()
        curses.endwin()
                
    def display_status(self):
        # Get status info
        try:
            self.status_info = get_status_info()
        except MemoryError:
            self.status_info = {}
            return
        # Write ststus info to display
        for key in status_fields:
            val = self.status_info[key]  
            if key in ['motor_ind', 'pwm_zero_ns', 'trig_index', 'trig_width']:
                outstr = '%s: '%(key,)
                for v in val:
                    outstr = outstr +  '%d, '%(v,)
            elif key == 'motor_type':
                outstr='%s: '%(key,)
                for v in val:
                    outstr = outstr+'%s, '%(v,)
            else:
                if type(val)==int:
                    outstr = '%s: %d'%(key,val)
                if type(val)==float:
                    outstr = '%s: %1.4f'%(key,val)
            try:
                self.stdscr.addstr(self.row, self.col+self.indent,outstr)
            except curses.error:
                pass
            self.row+=self.step
        self.stdscr.refresh()

    def display_header(self):
        self.stdscr.addstr(self.row,self.col,self.header_str)
        self.row+=self.step
        for i in range(0,30):
            try:
                self.stdscr.addch(self.row,i+self.col, curses.ACS_HLINE)
            except curses.error:
                pass
        self.row+=2*self.step
        self.stdscr.refresh()
           
# ------------------------------------------------------------------------
if __name__=='__main__':

    app = motor_status()
    app.run()
