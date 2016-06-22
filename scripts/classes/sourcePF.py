#!/usr/bin/env python

from math import *
import random
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import time

world_size = 100.0

class particlePlume:
    def __init__(self,realPlume = 0, numRobot=1):
        
        if realPlume: # if this is true, this initalizes my location of the real plume xs,ys and will also
                      # create my numRobot robots
            # location of my real plumen
            # these are my state variables
            self.ys = 50
            self.xs = 0
            self.Q  = .9
            self.v  = .8
            self.Dy = .75
            self.Dz = .5
            self.H  = 5
            # self.x  = self.xs + 80 # this is where I am sensing from
            # self.y  = self.ys + -40
            self.x=[]
            self.y=[]
            self.orientation = []
            self.numRobot = numRobot
            # print 'numRobot', self.numRobot
            for i in range(self.numRobot):
                self.x.append(random.random()*world_size)
                self.y.append(random.random()*world_size)
                self.orientation.append(random.random() * 2.0 * pi)
            # print 'orientation', self.orientation
        else:            # this is the simulated particles with random.random() 'guessing' the states
                         # notice that some of my states are given to me, but I could make them hidden

            # unknown variables or hidden states
            self.xs = random.random() * world_size # randomly set particle x location
            self.ys = random.random() * world_size # randomly set partlcle's y location
            self.Q  = random.random()
            self.v  = random.random()
            
            # known variables
            # self.Q  = .9#random.random()
            # self.v  = .8#random.random()
            self.Dy = .75#random.random()
            self.Dz = .5#random.random()
            self.H  = 5#random.random() * 5
            
            self.orientation = random.random() * 2.0 * pi
        
        # init noise parameter, will change later through method
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;
    
    def set(self, new_x, new_y, new_orientation):
        # this method sets a new location for the robots
        self.x = []
        self.y = []
        self.orientation = []
        if type(new_x) is list: # if this is true then new_x is the new position for the robot
            self.x = new_x
            self.y = new_y
            self.orientation = new_orientation
                
    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise): # sets the noises for self 
        # this includes the robots and the particles
        
        # variances
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);
        
    def move(self, turn, forward, myrobot=0):
                                
        # my particle is the source location, I am assuming this does not move
        dist = float(forward)# + random.gauss(0.0, self.forward_noise)
        if myrobot ==0:
            orientation = float(turn)
            orientation %= 2 * pi
            
            x = self.xs + (cos(orientation) * dist)
            y = self.ys + (sin(orientation) * dist)
            x %= world_size    # cyclic truncate
            y %= world_size
            res = particlePlume()
            res.set(x, y, orientation)
            res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
            return res
        else:
            # this is for my robot, it moves in a deterministic way            
            x = []
            y = []
            orientation = []
            for i in range(self.numRobot):
                # print 'turn',turn
                orientation.append(float(turn[i]))
                orientation[i] %= 2 * pi
                x.append(float((self.x[i] + (cos(orientation[i]) * dist))%world_size))
                y.append(float((self.y[i] + (sin(orientation[i]) * dist))%world_size))
            res = particlePlume(realPlume=1, numRobot=self.numRobot)
            res.set(x,y,orientation)
            res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
            return res
    
    def sense(self,myrobot):
        # get the concentration values for the robots locations
        Z = []
        conc = self.plume(myrobot)
        for i in range(myrobot.numRobot):
            conc[i] = conc[i]+random.gauss(0.0, self.sense_noise) # each sensor has additive noise
            Z.append(conc[i])
        return Z
    

    def plume(self,myrobot):
        Q = self.Q
        Dy = self.Dy
        Dz = self.Dz
        v = self.v
        H = self.H
        #  robot location - particle's estimate of source location 
        conc =[]
        for i in range(myrobot.numRobot): # I obtain numRobot observations
            xx = myrobot.x[i]-self.xs
            yy = myrobot.y[i]-self.ys
            conc.append(Q/(2*pi*xx*sqrt(Dy*Dz))*np.exp(-H**2*v/(4*Dz*xx))*np.exp(-v*yy**2/(4*Dy*xx)))

        # print 'particle at:', self.xs, self.ys
        # print 'concentration:', conc 
        
        return conc
    
    def Gaussian(self, mu, sigma, x):
        
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        prob = exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
        # print 'prob:', prob
        # print 'mu:', mu, 'sig:', sigma, 'x:', x
        return prob
    
    
    def measurement_prob(self, z, myrobot):
        
        # calculates how likely a measurement should be
        # z is my measurement

        conc = self.plume(myrobot)
        prob = 1
        for i in range(myrobot.numRobot):
            prob *= self.Gaussian(conc[i], self.sense_noise, z[i]) # product of all the sensed likelihoods
        # import pdb; pdb.set_trace()
        return prob
       
    def get_x(self):
        return self.xs
    def get_y(self):
        return self.ys
    def __repr__(self):
        return '[x=%.6s y=%.6s Q=%.6s]' % (str(self.xs), str(self.ys), str(self.Q))


def eval(r, p):
    sum = 0.0;
    for i in range(len(p)): # calculate mean error
        dx = (p[i].xs - r.xs + (world_size/2.0)) % world_size - (world_size/2.0)
        dy = (p[i].ys - r.ys + (world_size/2.0)) % world_size - (world_size/2.0)
        dq = (p[i].Q  - r.Q)
        dv = (p[i].v  - r.v)
        err = sqrt(dx * dx + dy * dy + dq * dq + dv * dv)
        sum += err
    return sum / float(len(p))

def get_state(particles):
    x = []
    y = []
    ori = []
    for p in particles:
        x.append(p.get_x())
        y.append(p.get_y())
    return (x,y)

def plot_PF(particles,plume, mp=None):
    plt.clf()
    plt.axis([-5,world_size+5,-5,world_size+5])
    xmin = 0.000000000
    xmax = world_size
    ymin = 0.000000000
    ymax = world_size
    resolution = 100
    
    [x,y] = np.meshgrid(np.linspace(xmin,xmax,resolution), np.linspace(ymin,ymax,resolution))

    Q = plume.Q
    Dy = plume.Dy
    Dz = plume.Dz
    v = plume.v
    H = plume.H
    xx = x-plume.xs
    yy = y-plume.ys
    conc=Q/(2*pi*xx*sqrt(Dy*Dz))*np.exp(-H**2*v/(4*Dz*xx))*np.exp(-v*yy**2/(4*Dy*xx))

    plt.contourf (x,y,conc)
    plt.colorbar()

    # plotting particles
    x,y = get_state(p)
    plt.scatter(x,y, alpha = .5, s = 25, color='k')#,s=w*100) # add the coloring to denote orientation or equivalent
    # plotting where the robot is
    plt.scatter(plume.x,plume.y,s = 50, color='r')
    plt.figtext(.16,.8, 'robot: ' +' x: '+ str(plume.x)+' y: '+ str(plume.y), color='w')
    # plotting where the real plume source is
    plt.scatter(plume.xs,plume.ys,color='y',s = 50, alpha = 1)
    plt.figtext(.16,.65, 'eval: ' + str(+eval(plume,particles)), color='w')

    if mp is not None:
        # atan2((myrobot.y-mp.ys),(myrobot.x-mp.xs))*180.0/pi
        plt.scatter(mp.xs,mp.ys,s=50, color='m')
        plt.figtext(.16,.75, 'mlp: ' +' xs: '+ str(mp.xs)+' ys: '+ str(mp.ys)+ ' Q: ' + str(mp.Q), color='w')
        plt.figtext(.16,.7,'mlp: ' +' v: ' + str(mp.v), color= 'w')
        # print atan2(mp.ys-myrobot.y,mp.xs-myrobot.x)*180.0/pi
    
    plt.draw()
    time.sleep(0.0001)
    


myrobot = particlePlume(realPlume=1, numRobot=2)
myrobot.set_noise(0.5,0.5,0.0001)
fig=plt.figure()
plt.ion()


N = 10000
T = 100
p = []
# initalizae all particles states randomly and set noise
for i in range(N):
    r = particlePlume(realPlume=0)
    r.set_noise(0.5,0.5,0.0001) # forward noise, turning noise, sensor noise
    p.append(r)
plot_PF(p,myrobot)
mp = None
for t in range(T):
    # direction = random.random()*2.0*pi
    print t
    if mp is not None: # init special case handling
        direction = []
        for i in range(myrobot.numRobot):
            direction.append(atan2(mp.ys-myrobot.y[i],mp.xs-myrobot.x[i]))
    else: # move in the direction of the most likely particle
        direction = []
        for i in range(myrobot.numRobot):
            direction.append(random.random()*2.0*pi)

    length = random.random()*5.0 # move a random length

    # move robot towards the most likely particle
    myrobot = myrobot.move(direction,length,myrobot=1)
    # obstain a new observation for each robot
    Z = myrobot.sense(myrobot)
        
    # reweight all the particles base off of the emmission model
    w = []
    for i in range(N):
        w.append(p[i].measurement_prob(Z,myrobot))
    

    # resample from the weighted particle distribution
    p3 = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)

    for i in range(N):
        # find the particle with the highest weigthing and print it out so I can see if it is converging to the source location i want
        if w[i] == mw:
            mp = p[i]
            # print i, p[i], w[i]
    
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append(p[index])
    p = p3

    plot_PF(p,myrobot, mp)
