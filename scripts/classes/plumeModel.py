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
    def get_grid(self):
        pass
    def get_conc(self):
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
        # return self.c
        return self.grid

    def main(self):
        # this is method is ran if this file is ran from the command line
        # it will show a matlab contour plot of the gaussian plume for the parameters defined in the if __name__ == main below
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        from matplotlib import cm
        from matplotlib.ticker import LinearLocator, FormatStrFormatter

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

        gradx, grady = np.gradient(c, self.resolution/len(x), self.resolution/len(x))

        fig1 = plt.figure()
        Q = plt.quiver(x, y,-grady,-gradx) # had to switch because of c.transpose and invert_xaxis to make it look like my rviz plots
        plt.gca().invert_xaxis()
        plt.gca().invert_yaxis()
        # plt.show()

        fig2 = plt.figure()
        plt.contourf (x,y,c)
        plt.colorbar()
        plt.gca().invert_xaxis()
        plt.gca().invert_yaxis()
        # plt.show()


        fig3 = plt.figure()
        ax = fig3.gca(projection='3d')
        surf = ax.plot_surface(x, y, c, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        plt.gca().invert_xaxis()
        plt.gca().invert_yaxis()
        plt.title('Concentration')
        plt.xlabel('x')
        plt.ylabel('y')
        # plt.show()

        # note I hade to switch the data and label because of the invert xaxis and yaxis to look like my rviz plots
        fig4 = plt.figure()
        ax = fig4.gca(projection='3d')
        surf = ax.plot_surface(x, y, np.abs(gradx), rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        plt.gca().invert_xaxis()
        plt.gca().invert_yaxis()
        plt.title('abs of gradient Y')
        plt.xlabel('x')
        plt.ylabel('y')
        # plt.show()

        fig5 = plt.figure()
        ax = fig5.gca(projection='3d')
        surf = ax.plot_surface(x, y,np.abs(grady), rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        plt.gca().invert_xaxis()
        plt.gca().invert_yaxis()
        plt.title('abs gradient X')
        plt.xlabel('x')
        plt.ylabel('y')
        # plt.show()
        
        # gradx = np.zeros(100,100)
        # grady = np.ones(100,100)

        gradM = np.power(np.add(np.power(gradx,2),np.power(grady,2)),.5)

        fig6 = plt.figure()
        ax = fig6.gca(projection='3d')
        surf = ax.plot_surface(x, y, gradM, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        plt.gca().invert_xaxis()
        plt.gca().invert_yaxis()
        plt.title('gradient')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.show()

class linearPlume(plumeModel): # child class

    def __init__(self,originX=0, originY=0, resolution=100, width=100, height=100, a=1,b=1,c=1,d=1):
        self.originX = originX
        self.originY = originY
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.zeros((resolution,resolution))
        self.a = a
        self.b = b
        self.p = c
        self.d = d
        
        super(linearPlume, self).__init__(originX, originY, resolution,width, height)
        # this line of code utilizes the init method from the previous class plumeModel
        
        # print self.resolution, self.width, self.height
        
        self.set_values() # this method sets the concentration values and maps the values from 0-1 to self.grid

    def set_values(self):
        xmin = 1
        ymin = 1
        [x,y] = np.meshgrid(np.linspace(xmin,self.width,num=self.resolution), np.linspace(ymin,self.height\
                                                                                      ,num=self.resolution))
        # d = 1
        # a = 1
        # b = 1
        # c = 10

        Z=(self.d- self.a*x - self.b*y)/self.p;
        
        self.c = Z
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
        from mpl_toolkits.mplot3d import Axes3D
        from matplotlib import cm
        from matplotlib.ticker import LinearLocator, FormatStrFormatter


        xmin = 1
        ymin = 1
        [x,y] = np.meshgrid(np.linspace(xmin,self.width,num=self.resolution), np.linspace(ymin,self.height\
                                                                                      ,num=self.resolution))

        Z=(self.d - self.a*x - self.b*y)/self.p

        # Z = (self.d-self.a*x - self.b*y)/self.c        
        c = Z
        c = np.transpose(c)

        # gradx, grady = np.gradient(c, self.resolution/len(x), self.resolution/len(x))

        # fig1 = plt.figure()
        # Q = plt.quiver(x, y,-grady,-gradx) # had to switch because of c.transpose and invert_xaxis to make it look like my rviz plots
        # plt.gca().invert_xaxis()
        # plt.gca().invert_yaxis()
        # # plt.show()

        # fig2 = plt.figure()
        # plt.contourf (x,y,c)
        # plt.colorbar()
        # plt.gca().invert_xaxis()
        # plt.gca().invert_yaxis()
        # plt.show()


        fig3 = plt.figure()
        ax = fig3.gca(projection='3d')
        surf = ax.plot_surface(x, y, c, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        plt.gca().invert_xaxis()
        plt.gca().invert_yaxis()
        plt.title('Concentration')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.show()

        # # note I hade to switch the data and label because of the invert xaxis and yaxis to look like my rviz plots
        # fig4 = plt.figure()
        # ax = fig4.gca(projection='3d')
        # surf = ax.plot_surface(x, y, np.abs(gradx), rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        # plt.gca().invert_xaxis()
        # plt.gca().invert_yaxis()
        # plt.title('abs of gradient Y')
        # plt.xlabel('x')
        # plt.ylabel('y')
        # # plt.show()

        # fig5 = plt.figure()
        # ax = fig5.gca(projection='3d')
        # surf = ax.plot_surface(x, y,np.abs(grady), rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        # plt.gca().invert_xaxis()
        # plt.gca().invert_yaxis()
        # plt.title('abs gradient X')
        # plt.xlabel('x')
        # plt.ylabel('y')
        # # plt.show()
        
        # # gradx = np.zeros(100,100)
        # # grady = np.ones(100,100)

        # gradM = np.power(np.add(np.power(gradx,2),np.power(grady,2)),.5)

        # fig6 = plt.figure()
        # ax = fig6.gca(projection='3d')
        # surf = ax.plot_surface(x, y, gradM, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        # plt.gca().invert_xaxis()
        # plt.gca().invert_yaxis()
        # plt.title('gradient')
        # plt.xlabel('x')
        # plt.ylabel('y')
        # plt.show()


class randomPlume(plumeModel): # child class

    def __init__(self,originX=0, originY=0, resolution=100, width=100, height=100, h=1, clx=50):
        self.originX = originX
        self.originY = originY
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.zeros((resolution,resolution))
        self.h = h # rms height
        self.clx = clx
        # [f,x,y] = rsgene2D(N,rL,h,clx,cly) 

        super(randomPlume, self).__init__(originX, originY, resolution,width, height)
        # this line of code utilizes the init method from the previous class plumeModel
        
        # print self.resolution, self.width, self.height
        
        self.set_values() # this method sets the concentration values and maps the values from 0-1 to self.grid

    def set_values(self):
        xmin = 0
        ymin = 0
        
        
        xmin = 0
        ymin = 0
        x = np.linspace(xmin,self.width,num=self.resolution)
        y = np.linspace(xmin,self.height,num=self.resolution)
        [X,Y] = np.meshgrid(np.linspace(xmin,self.width,num=self.resolution), np.linspace(ymin,self.height\
                                                                                      ,num=self.resolution))

        # x = linspace(-rL/2,rL/2,N)
        # y = linspace(-rL/2,rL/2,N)
        # [X,Y] = meshgrid(x,y)

        N = self.resolution # number of points
        rL = self.width #surcface displacement
        # clx = 50 # increase for smoother
        iso = 1 # isotropic surface

        if iso==0:
            cly = self.clx
        # cly = 1 # isotropic if commented out

        Z = self.h*np.random.randn(N,N) #% uncorrelated Gaussian random rough surface distribution
                           # % with rms height h
        # % isotropic surface                           
        if iso == 1:
            # % Gaussian filter
            F = np.exp(-(abs(X)+abs(Y))/(self.clx/2))
            # % correlation of surface including convolution (faltung), inverse
            # % Fourier transform and normalizing prefactors
            f = 2.0*rL/N/self.clx*np.fft.ifft2(np.fft.fft2(Z)*np.fft.fft2(F))
            # print f
            # print f.real
            # % non-isotropic surface
        elif iso == 0:
            # % Gaussian filter
            F = np.exp(-(abs(X)/(self.clx/2)+abs(Y)/(cly/2)))
            # % correlation of surface including convolution (faltung), inverse
            # % Fourier transform and normalizing prefactors
            f = 2.0*rL/N/sqrt(self.clx*cly)*np.fft.ifft2(np.fft.fft2(Z)*np.fft.fft2(F))


        self.c = f.real
        # self.c =np.transpose(self.c)
                
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
        from mpl_toolkits.mplot3d import Axes3D
        from matplotlib import cm
        from matplotlib.ticker import LinearLocator, FormatStrFormatter

        xmin = 0
        ymin = 0
        x = np.linspace(xmin,self.width,num=self.resolution)
        y = np.linspace(xmin,self.height,num=self.resolution)
        [X,Y] = np.meshgrid(np.linspace(xmin,self.width,num=self.resolution), np.linspace(ymin,self.height\
                                                                                      ,num=self.resolution))

        # x = linspace(-rL/2,rL/2,N)
        # y = linspace(-rL/2,rL/2,N)
        # [X,Y] = meshgrid(x,y)

        N = self.resolution # number of points
        rL = self.width #surcface displacement
        # clx = 50 # increase for smoother
        iso = 1 # isotropic surface
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        if iso==0:
            cly = 1
        # cly = 1 # isotropic if commented out

        Z = self.h*np.random.randn(N,N) #% uncorrelated Gaussian random rough surface distribution
                           # % with rms height h

        # % isotropic surface                           
        if iso == 1:
            # % Gaussian filter
            F = np.exp(-(abs(X)+abs(Y))/(self.clx/2))
            # % correlation of surface including convolution (faltung), inverse
            # % Fourier transform and normalizing prefactors
            f = 2.0*rL/N/self.clx*np.fft.ifft2(np.fft.fft2(Z)*np.fft.fft2(F))
            # print f
            # print f.real
            # % non-isotropic surface
        elif iso == 0:
            # % Gaussian filter
            F = np.exp(-(abs(X)/(self.clx/2)+abs(Y)/(cly/2)))
            # % correlation of surface including convolution (faltung), inverse
            # % Fourier transform and normalizing prefactors
            f = 2.0*rL/N/sqrt(self.clx*cly)*np.fft.ifft2(np.fft.fft2(Z)*np.fft.fft2(F))
            print 'hi'
            
        # print x.imag
        # print y.imag
        ax = fig.gca(projection='3d')
        surf = ax.plot_surface(X, Y, f.real, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        # plt.gca().invert_xaxis()
        # plt.gca().invert_yaxis()
        # plt.title('Concentration')
        # plt.xlabel('x')
        # plt.ylabel('y')
        plt.show()

# % [f,x,y] = rsgene2D(N,rL,h,clx,cly) 
# %
# % generates a square 2-dimensional random rough surface f(x,y) with NxN 
# % surface points. The surface has a Gaussian height distribution and 
# % exponential autocovariance functions (in both x and y), where rL is the 
# % length of the surface side, h is the RMS height and clx and cly are the 
# % correlation lengths in x and y. Omitting cly makes the surface isotropic.
# %
# % Input:    N   - number of surface points (along square side)
# %           rL  - length of surface (along square side)
# %           h   - rms height
# %           clx, (cly)  - correlation lengths (in x and y)
# %
# % Output:   f  - surface heights
# %           x  - surface points
# %           y  - surface points
# %

# %

# format long;

# x = linspace(-rL/2,rL/2,N); y = linspace(-rL/2,rL/2,N);
# [X,Y] = meshgrid(x,y); 

# Z = h.*randn(N,N); % uncorrelated Gaussian random rough surface distribution
#                    % with rms height h
                   
# % isotropic surface
# if nargin == 4
#     % Gaussian filter
#     F = exp(-(abs(X)+abs(Y))/(clx/2));
    
#     % correlation of surface including convolution (faltung), inverse
#     % Fourier transform and normalizing prefactors
#     f = 2*rL/N/clx*ifft2(fft2(Z).*fft2(F));
    
# % non-isotropic surface
# elseif nargin == 5
#     % Gaussian filter
#     F = exp(-(abs(X)/(clx/2)+abs(Y)/(cly/2)));
    
#     % correlation of surface including convolution (faltung), inverse
#     % Fourier transform and normalizing prefactors
#     f = 2*rL/N/sqrt(clx*cly)*ifft2(fft2(Z).*fft2(F));
    
# end

if __name__ == "__main__":
    # myPlume = gaussPlume(xs=0,ys=50,Q=1,Dy=1,Dz=.5,v=1,h=5,originX=0,originY=0,resolution=100,width=100,height=100)
    # print myPlume.c, np.size(myPlume.c,0),np.size(myPlume.c,1)
    # myPlume.main() # this plots the plume
    myPlume = randomPlume(resolution=100,width=100,height=100,h=5,clx=1000)
    myPlume.main()
