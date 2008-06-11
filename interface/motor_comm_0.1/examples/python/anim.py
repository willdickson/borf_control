#!/usr/bin/env python

# if 0:
#     from pylab import *
#     import time
#     ion() 
#     tstart = time.time()               # for profiling
#     x = arange(0,2*pi,0.01)            # x-array
#     line, = plot(x,sin(x))
#     for i in arange(1,200):
#         line.set_ydata(sin(x+i/10.0))  # update the data
#         draw()                         # redraw the canvas

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
        self.T = 5.0
        self.N = 1000
        self.dt = self.T/float(self.N)
        self.t = scipy.arange(0,self.N)*self.dt - self.T
        self.x = list(scipy.zeros(self.t.shape))
        self.ax = p.subplot(111)
        self.canvas = self.ax.figure.canvas
        
        self.line, = p.plot(self.t,self.x,animated=True)
        
        print self.t.min(), self.t.max()

        p.ylim(0,4095) # Want to get this from maxdata
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
        self.canvas.restore_region(self.background)
        # Get new data point
        ain_data = self.comm.get_ain_data()
        new_x = ain_data[0]
        #print new_x

        self.x.remove(self.x[0])
        self.x.append(new_x)
        
        # Plot new point
        self.line.set_ydata(self.x)
      
        self.ax.draw_artist(self.line)
        self.canvas.blit(self.ax.bbox)

        time.sleep(self.dt)
     
        return True

#    def clean_up(self):
#        print 'closing'
#        self.comm.close()
             
        

app = StripChart()
app.run()
        




