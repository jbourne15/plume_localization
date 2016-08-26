#!/usr/bin/env python

# ------------------------------------------------------------------

# Now we want to give weight to our 
# particles. This program will print a
# list of 1000 particle weights.
#
# Don't modify the code below. Please enter
# your code at the bottom.

from math import *
import random
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import time

# Now add noise to your robot as follows:
# forward_noise = 5.0, turn_noise = 0.1,
# sense_noise = 5.0.
#
# Once again, your robot starts at 30, 50,
# heading north (pi/2), then turns clockwise
# by pi/2, moves 15 meters, senses,
# then turns clockwise by pi/2 again, moves
# 10 m, then senses again.
#
# Your program should print out the result of
# your two sense measurements.
#
# Don't modify the code below. Please enter
# your code at the bottom.

from math import *
import random



landmarks  = [[1.0, 1.0], 
              [99.0, 99.0], 
              [1.0, 99.0],
              [99.0, 1.0]]#,
              # [50.0, 40.0]]

world_size = 100.0


class robot:
    def __init__(self):
        self.x = random.random() * world_size # randomly set particle x location
        self.y = random.random() * world_size # randomly set partlcle's y location
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;
    
    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError, 'X coordinate out of bound'
        if new_y < 0 or new_y >= world_size:
            raise ValueError, 'Y coordinate out of bound'
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);
    
    
    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z
    
    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError, 'Robot cant move backwards'         
        
        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi
        
        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size    # cyclic truncate
        y %= world_size
        
        # set particle
        res = robot()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res
    
    def Gaussian(self, mu, sigma, x):
        
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        prob = exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
        # print 'mu:', mu, 'sigma:',  sigma, 'x:',  x, 'prob:', prob
        # raw_input()
        return prob
    
    
    def measurement_prob(self, measurement):
        
        # calculates how likely a measurement should be
        prob = 1.0 
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i]) # this has assumed sensor noise 
        return prob
    
    def get_x(self):
        return self.x
    def get_y(self):
        return self.y
    def get_ori(self):
        return self.orientation
    
    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))

def eval(r, p):
    sum = 0.0;
    for i in range(len(p)): # calculate mean error
        dx = (p[i].x - r.x + (world_size/2.0)) % world_size - (world_size/2.0)
        dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
        err = sqrt(dx * dx + dy * dy)
        sum += err
    return sum / float(len(p))

def get_state(particles):
    x = []
    y = []
    ori = []
    for p in particles:
        x.append(p.get_x())
        y.append(p.get_y())
        ori.append(p.get_ori())
    return (x,y,ori)

def plot_PF(particles,robot):
    plt.clf()
    plt.axis([-20,120,-20,120])
    x,y,ori = get_state(p)
    plt.scatter(x,y, alpha = .5, s = 25)#,s=w*100) # add the coloring to denote orientation or equivalent
    landx = []
    landy = []
    for i,j in landmarks:
        landx.append(i)
        landy.append(j)
    # print landx, landy
    plt.scatter(landx,landy,color='r',s=50, alpha = .8)
    plt.scatter(myrobot.get_x(),myrobot.get_y(),color='m',s = 75, alpha = 1)
    # ori = robot.get_ori()
    # print 'ori', ori
    # xp,yp = ori[0]*2, ori()[1]*2

    # plt.arrow(myrobot.get_x(),myrobot.get_y(),2,2, head_width=1, head_length=1, fc='m', ec='m')
    
    # print str(eval(robot,particles))
    plt.figtext(.25,.75, 'eval: ' + str(+eval(robot,particles)))
    # circle=plt.Circle((myrobot.get_x(),myrobot.get_y()),eval(myrobot,particles))
    # fig = plt.gcf()
    # fig.gca().add_artist(circle)
    
    plt.draw()
    time.sleep(.0001)
    


####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####

myrobot = robot()
real_noise = 1.0
predicted_noise = 1.0
myrobot = myrobot.move(0.1, 5.0)
myrobot.set_noise(0.5,0.5,real_noise)

Z = myrobot.sense()
fig=plt.figure()
plt.ion()


N = 2000
T = 100
p = []
for i in range(N):
    r = robot()
    r.set_noise(0.5, 0.5, predicted_noise)
    p.append(r)
plot_PF(p,myrobot)
for t in range(T):
    myrobot = myrobot.move(0.1, 5.0)
    Z = myrobot.sense()

    # state transition prediction
    p2 = []
    for i in range(N):
        p2.append(p[i].move(0.1, 5.0))
    p = p2
    plot_PF(p,myrobot)
    
    # observational update
    w = []
    for i in range(N):
        w.append(p[i].measurement_prob(Z))

    # Resampling weighted particles
    p3 = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append(p[index])
    p = p3
    plot_PF(p,myrobot)
