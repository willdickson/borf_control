#!/usr/bin/env python

#     print 'FPS:' , 200/(time.time()-tstart)
import sys
import gtk, gobject
import pylab as p
import scipy
import time
import atexit
from motor_comm import Motor_Comm


class StripChart:

    def __init__(self):
        self.comm = Motor_Comm()
        #atexit.register(self.clean_up)
        self.T = 2.0
        self.N = 1000
        self.dt = self.T/float(self.N)
        self.t = scipy.arange(0,self.N)*self.dt - self.T
        self.x = list(scipy.zeros(self.t.shape))
        self.ax = p.subplot(111)
        self.canvas = self.ax.figure.canvas
        self.started = False

        self.line, = p.plot(self.t,self.x,animated=True)
        
        print self.t.min(), self.t.max()

        p.ylim(-10,10) # Want to get this from maxdata
        p.xlim(self.t.min(),self.t.max())
        p.xlabel('t (sec)')
        p.ylabel('(V)')
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)
        
    
        p.connect('resize_event', self.resize)

    def resize(self, *args):
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)
        

    def run(self):
        gobject.idle_add(self.update_line)
        p.show()
        


    def update_line(self,*args):
        if self.started == False:
            self.ax.figure.canvas.draw()
            self.started=True
        self.canvas.restore_region(self.background)
        # Get new data point
        ain_data = self.comm.get_ain_data()
        new_x = ain_data[0]
        
        self.x.remove(self.x[0])
        self.x.append(new_x)
        
        # Plot new point
        self.line.set_ydata(self.x)
      
        self.ax.draw_artist(self.line)
        self.canvas.blit(self.ax.bbox)

        time.sleep(self.dt)
     
        return True

                     

app = StripChart()
app.run()
        




