#!/usr/bin/env python

import numpy as np
import math

#ros specific 
# import rospy
# from std_msgs.msg import String

class plumeModel(object): # parent class

    def __init__(self,  originX=0, originY=0, resolution=10,width=50, height=50):
        self.originX = originX
        self.originY = originY
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.zeros((resolution,resolution))
    
    def set_values(self):
        # Placeholder for inherited class specific to the type of model being used
        pass

    def remapping(self,x, inmin, inmax, outmin, outmax):
        # This method remaps inputs to a value between outmin and outmax
        return (x-inmin)*(outmax-outmin)/(inmax-inmin)+outmin

class gaussPlume(plumeModel): # child class

    def __init__(self, xs=0, ys=50, Q=1, Dy=1, Dz=.5, v=1,h=5, originX=0, originY=0, resolution=100,\
                 width=50, height=50):
        # gaussPlume constructor==
        # xs, ys, Q, Dy, Dz, v, h, originX, originY, resolution, width, height

        self.xs = xs
        self.ys = ys
        if xs>width:
            self.xs = width/2
            print 'error in stack position'
        elif ys>height:
            self.ys = height/2
            print 'error in stack position'
        self.Q = Q
        self.Dy = Dy
        self.Dz = Dz
        self.v = v
        self.h = h
        super(gaussPlume, self).__init__(originX, originY, resolution,width, height)
        # this line of code utilizes the init method from the previous class plumeModel
        
        self.set_values() # this method sets the concentration values and maps the values from 0-1 to self.grid

    def set_values(self):
        xmin = 1
        ymin = 1
        [x,y] = np.meshgrid(np.linspace(xmin,self.width,num=self.resolution), np.linspace(ymin,self.height\
                                                                                      ,num=self.resolution))
        xx = x - self.xs
        yy = y - self.ys
        self.c = self.Q/(2*np.pi*xx*math.sqrt(self.Dy*self.Dz))\
                      *np.exp(-self.h**2*self.v/(4*self.Dz*xx))*np.exp(-self.v*yy**2/(4*self.Dy*xx))
        self.c =np.transpose(self.c)
        
        
        Cmax = np.ndarray.max(self.c)
        Cmin = np.ndarray.min(self.c)

        # map to the 0-100 range for the Occupancygrid msg
        for i in range(0,len(self.c)):
            for j in range(0,len(self.c)):
                self.grid[i,j] = self.remapping(self.c[i,j], Cmin, Cmax, 0, 100)

    def get_grid(self):
        return self.grid
    def get_conc(self):
        return self.c

    def main(self):
        # this is method is ran if this file is ran from the command line
        # it will show a matlab contour plot of the gaussian plume for the parameters defined in the if __name__ == main below
        import matplotlib.pyplot as plt

        Q = self.Q

        xs = self.xs
        ys = self.ys
        
        xmin = 0.000000001
        xmax = self.width
        ymin = 0.000000001
        ymax = self.height

        [x,y] = np.meshgrid(np.linspace(xmin,xmax,self.resolution), np.linspace(ymin,ymax,self.resolution))
        Dy = self.Dy
        Dz = self.Dy
        v = self.v
        H = self.h
        xx = x-xs
        yy = y-ys
        
        c=Q/(2*np.pi*xx*math.sqrt(Dy*Dz))*np.exp(-H**2*v/(4*Dz*xx))*np.exp(-v*yy**2/(4*Dy*xx))
        
        c = np.transpose(c)
        
        plt.contourf (x,y,c)
        plt.colorbar()
        plt.show()

if __name__ == "__main__":
    myPlume = gaussPlume(xs=0,ys=5,Q=1,Dy=1,Dz=.5,v=1,h=5,originX=0,originY=0,resolution=4,width=10,height=10)
    print myPlume.c, np.size(myPlume.c,0),np.size(myPlume.c,1)
    myPlume.main() # this plots the plume
