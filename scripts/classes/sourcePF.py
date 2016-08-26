#!/usr/bin/env python
from math import *
import random
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import time
import matplotlib
from matplotlib import cm


world_size = 200.0

class particlePlume:
    def __init__(self,realPlume = 0, numRobot=1):
        
        if realPlume: # if this is true, this initalizes my location of the real plume xs,ys and will also
                      # create my numRobot robots
            # location of my real plumen
            # these are my state variables
            self.ys = 120
            self.xs = 70
            # self.ys = 50
            # self.xs = 0
            self.Q  = 1.9
            self.v  = .75
            self.Dy = 2.0
            self.Dz = .6
            self.H  = 5
            self.a = 3.6*pi/2.0
            # self.a = 3.0*pi/2.0

            # problematic
            # self.a = pi/2.0 
            # self.ys = 30
            # self.xs = 20

            # poor performance
            # self.a = 3.6*pi/2.0
            # self.ys = 30
            # self.xs = 20

            # self.a = 0
            self.x=[]
            self.y=[]
            self.lastC=[]
            
            self.msg = []
            
            self.orientation = []
            self.numRobot = numRobot
            # self.conc_thres = 0.001
            self.conc_thres = 0.0015

            for i in range(self.numRobot):
                # self.x.append(random.random()*world_size)
                # self.y.append(random.random()*world_size)
                self.y.append(10.0)
                self.x.append(160.0-i*40.0)
                # self.orientation.append(random.random() * 2.0 * pi)
                self.lastC.append(None)
                # print 'setting lastC to None'
                self.orientation.append(pi/2.0)
                self.msg.append('init')
            # print 'orientation', self.orientation
        else:            # this is the simulated particles with random.random() 'guessing' the states
                         # notice that some of my states are given to me, but I could make them hidden

            # unknown variables or hidden states
            self.xs = random.random() * world_size # randomly set particle x location
            self.ys = random.random() * world_size # randomly set partlcle's y location
            self.Q  = random.random()*2.5 + .5
            self.v  = random.random()*2.0 + .25
            self.Dy = random.random() + 1.5
            self.Dz = random.random() + .5
            self.H  = random.random() * 10 + 2
            self.a  = random.random() * 2*pi
        
        # init noise parameter, will change later through method
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;
    
    def set(self, new_x, new_y, new_orientation,lastC, msg):#lastC
        # this method sets a new location for the robots
        self.x = []
        self.y = []
        self.orientation = []
        if type(new_x) is list: # if this is true then new_x is the new position for the robot
            self.x = new_x
            self.y = new_y
            self.orientation = new_orientation
            self.lastC = lastC
            self.msg = msg
    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise): # sets the noises for self 
        # this includes the robots and the particles
        # variances
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);

    def move_particle(self,w,dflag):
        # print 'moved particle'
        res = particlePlume()
        sig_xy  = 1
        # sig_QvDyDz = .025
        sig_QDyDz = .005
        sig_H = .1
        sig_v  = .025
        sig_a = 5*pi/180.0
        
        # increase standard deviation when I am in a degenerative case
        # what I am doing here doesn't always works because
        # I should sample in the direction of more likely particles
        # this could be a cool idea: keep estimate of the direction of more likely particles, so I bias my sample towards this direction.
        dflag = dflag*5
        
        # I allow for the sensitive parameter to shift more once I am in degeneracy
        sig_xy += dflag*sig_xy
        sig_QDyDz += dflag/3.0*sig_QDyDz
        sig_H += dflag/2.0*sig_H
        sig_a += dflag*sig_a
        sig_v += dflag*sig_v

        res.xs  = random.gauss(self.xs,sig_xy)
        res.ys  = random.gauss(self.ys,sig_xy)

        while True:
            res.Q = random.gauss(self.Q,sig_QDyDz)
            if res.Q>0:
                break
        while True:
            res.v  = random.gauss(self.v,sig_v)
            if res.v>0:
                break
        while True:
            res.Dy  = random.gauss(self.Dy,sig_QDyDz)
            if res.Dy>0:
                break
        while True:
            res.Dz  = random.gauss(self.Dz,sig_QDyDz)
            if res.Dz>0:
                break
        while True:
            res.H  = random.gauss(self.H,sig_H)
            if res.H>0:
                break
        while True:
            res.a = random.gauss(self.a,sig_a)#%(2.0*pi)
            # if res.a>2*pi:
                # res.a = res.a%(2.0*pi)
            if res.a>0:
                break
            
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        
        return res
        
    def move(self, turn, forward, myrobot=0):
                                
        # my particle is the source location, I am assuming this does not move
        dist = float(forward)# + random.gauss(0.0, self.forward_noise)
        if myrobot ==0:
            # orientation = float(turn)
            # orientation %= 2 * pi
                     
            # not using this, using move_particle
            self.xs = np.random.normal(self.xs, sigma, 1)
            self.ys = np.random.normal(self.xs, sigma, 1)
            self.Q = np.random.normal(self.Q, sigma, 1)
            self.v = np.random.normal(self.v, sigma, 1)
            self.Dy = np.random.normal(self.Dy, sigma, 1)
            self.Dz = np.random.normal(self.Dz, sigma, 1)
            self.H = np.random.normal(self.H, sigma, 1)

            res = particlePlume()
            res.set(x, y, orientation, myrobot.msg) # set new particle states
            res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
            return res
        else:
            # this is for my robot, it moves in a deterministic way            
            x = []
            y = []
            orientation = []

            # angle = [(b*180.0/pi)%360 for b in turn]
            # print 'moving in', angle
            
            for i in range(self.numRobot):
                # print 'turn',turn
                orientation.append(float(turn[i]))
                orientation[i] %= 2 * pi
                x.append(float((self.x[i] + (cos(orientation[i]) * dist))%world_size))
                y.append(float((self.y[i] + (sin(orientation[i]) * dist))%world_size))
            
            res = particlePlume(realPlume=1, numRobot=self.numRobot)
            res.set(x,y,orientation,self.lastC,self.msg)
            res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
            return res
    
    def sense(self,myrobot):
        # get the concentration values for the robots locations
        Z = []
        conc = self.plume(myrobot)
        for i in range(myrobot.numRobot):
            noise = random.gauss(0.0, self.sense_noise)
            while abs(noise)>conc[i] and noise<0: # make sure that the noise doesn't make the sensor reading go negative
                noise = random.gauss(0.0, self.sense_noise)

            process_noise = 0.0 #random.gauss(.001,0.05)
            # need to add the angle hereas well for sensing the real plume

            # distortion = np.sin((self.x[i]*1/40.0)*(self.y[i]*1/40.0))/1.0 + process_noise # large meadering plume
            # distortion = np.sin((self.x[i]*1/20.0)) # short plume
            # distortion = np.cos((self.y[i]*1/30.0))*np.sin((self.x[i]*1/30.))+0.6 # meadering plume
            # distortion = np.cos((self.y[i]*1/25.0))*np.sin((np.exp(self.x[i])*1/25.))+5.0 # noisy gaussian plume

            distortion = 1.0
            
            distortion = distortion + process_noise

            # if conc[i]*distortion+noise < self.conc_thres: # I can only sense after a certain threshold 
            #     conc[i] = 0
            # else:
            #     # distortion = 1.0
            #     conc[i] = conc[i] * distortion + noise
            
            # replaced with 
            conc[i] = conc[i] * distortion + noise

            Z.append(conc[i])
        return Z
    

    def plume(self,myrobot):
        Q = self.Q
        Dy = self.Dy
        Dz = self.Dz
        v = self.v
        H = self.H
        a = self.a

        #  robot location - particle's estimate of source location 
        conc =[]
        for i in range(myrobot.numRobot): # I obtain numRobot observations
            xx = myrobot.x[i]-self.xs # assuming I have these locations perfectly
            yy = myrobot.y[i]-self.ys # assuming I have these locations perfectly
            test_point = np.matrix([[xx],[yy]])
            R  = np.matrix([[cos(a), -sin(a)],[sin(a),cos(a)]])
            tp = np.transpose(R)*test_point
            xx = tp[0,0]
            yy = tp[1,0]
            
            if xx<0:
                conc.append(0)
                # print 'sensing behind the source'
            else:
                c = Q/(2*pi*xx*sqrt(Dy*Dz))*np.exp(-H**2*v/(4*Dz*xx))*np.exp(-v*yy**2/(4*Dy*xx))
                conc.append(c)
                
        return conc
    
    def Gaussian(self, mu, sigma, x):
        
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        prob = exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) #/ sqrt(2.0 * pi * (sigma ** 2))
        # print 'prob:', prob
        # print 'mu:', mu, 'sig:', sigma, 'x:', x
        return prob
    
    
    def measurement_prob(self, z, myrobot):
        
        # calculates how likely a measurement should be
        # z is my measurement

        conc = self.plume(myrobot) # this gives me the conc for the particle self at my robot's location
        prob = 1
        for i in range(myrobot.numRobot):
            cflag =1
            if cflag: # If i have seen my first concentration then update particles weight
                # this essentially waits for the first concentratin measurement to be seen before 
                # i start reweighting particles

                if True: #z[i] >= myrobot.conc_thres: # don't update if I sense nothing, sensing nothing can be miss leading for realistic plumes
                    if debug:
                        print 'conc', conc[i], 'sensed', z[i], 'stdev', self.sense_noise
                        print 'gaussian', self.Gaussian(conc[i], self.sense_noise, z[i]) # should this be giving values bigger than 1???????
# ...apparently it can http://math.stackexchange.com/questions/105455/how-can-a-probability-density-be-greater-than-one-and-integrate-to-one 
                    prob *= self.Gaussian(conc[i], self.sense_noise, z[i]) # product of all the sensed likelihoods
        if debug:
            print 'prob', prob


        if prob == 1: # for when i dont update if i sense nothing then reset my prob
            prob = 1.0/N
        return prob
       
    def get_x(self):
        return self.xs
    def get_y(self):
        return self.ys
    def __repr__(self):
        return '[x=%.6s y=%.6s]' % (str(self.xs), str(self.ys))


def eval(r, p):
    # sum = 0.0;
    # for i in range(len(p)): # calculate mean error
    #     dx = (p[i].xs - r.xs + (world_size/2.0)) % world_size - (world_size/2.0)
    #     dy = (p[i].ys - r.ys + (world_size/2.0)) % world_size - (world_size/2.0)
    #     dq = (p[i].Q  - r.Q)
    #     dv = (p[i].v  - r.v)
    #     dDy = (p[i].Dy - r.Dy)
    #     dDz = (p[i].Dz - r.Dz)
    #     dH = (p[i].H - r.H)
    #     err = sqrt(dx * dx + dy * dy + dq * dq + dv * dv + dDy*dDy + dDz*dDz + dH*dH)
    #     sum += err
    # return sum / float(len(p))

    x,y,Q,v,Dy,Dz,H,a = get_states(p)
    sxs = sum(x)/len(x)
    sys = sum(y)/len(y)
    sQ = sum(Q)/len(Q)
    sv = sum(v)/len(v)
    sDy = sum(Dy)/len(Dy)
    sDz = sum(Dz)/len(Dz)
    sH = sum(H)/len(H)

    y_unit = 0
    x_unit = 0
    for i in range(len(a)):
        y_unit += np.sin(a[i])
        x_unit += np.cos(a[i])
    sa = np.arctan2(y_unit,x_unit)  # circular mean for angles


    dx = (sxs - r.xs + (world_size/2.0)) % world_size - (world_size/2.0)
    dy = (sys - r.ys + (world_size/2.0)) % world_size - (world_size/2.0)
    dq = (sQ  - r.Q)
    dv = (sv  - r.v)
    dDy = (sDy - r.Dy)
    dDz = (sDz - r.Dz)
    dH = (sH - r.H)
    da = (sa - r.a)

    err = sqrt(dx*dx + dy*dy + dq*dq + dv*dv + dDy*dDy + dDz*dDz + dH*dH + da*da)

    return err

def get_states(particles):
    xs = []
    ys = []
    Q = []
    v = []
    Dy = []
    Dz = []
    H = []
    a = []
    for p in particles:
        xs.append(p.get_x())
        ys.append(p.get_y())
        Q.append(p.Q)
        v.append(p.v)
        Dy.append(p.Dy)
        Dz.append(p.Dz)
        H.append(p.H)
        a.append(p.a%(2.0*pi))

    return (xs,ys, Q, v, Dy, Dz, H, a)

def plot_PF(particles,plume, mp=None, w=None):
    # right now I am replotting the stationary plume this is slow

    plt.figure(1)
    plt.clf()

    plt.axis([-world_size/2.0,world_size+world_size/2.0,-world_size/2.0,world_size+world_size/2.0])

    # plt.axis([world_size/1.5-world_size,world_size+world_size/1.5,world_size/1.5-world_size,world_size+world_size/1.5])

    xmin = 0.000000000
    xmax = world_size*2
    ymin = 0.000000000
    ymax = world_size*2
    resolution = 100
    
    [x,y] = np.meshgrid(np.linspace(xmin,xmax,resolution), np.linspace(ymin,ymax,resolution))

    Q = plume.Q
    Dy = plume.Dy
    Dz = plume.Dz
    v = plume.v
    H = plume.H

    # Q = .9
    # Dy = .75
    # Dz = .5
    # v = .8
    # H = 5
    
    xx = x#-plume.xs
    yy = y-plume.ys

    conc=1.0*Q/(2.0*pi*xx*sqrt(Dy*1.0*Dz))*np.exp(-H**2.0*v*1.0/(4.0*Dz*xx))*np.exp(-v*yy**2.0/(4.0*Dy*xx))

    # distortion = np.sin((x*1/40.0)*(y*1/40.0))/1.0 # large meadering plume
    # distortion = np.sin((x*1/20.0)) # short plume
    # distortion = np.cos((y*1/30.0))*np.sin((x*1/30.))+0.6 # meadering plume
    # distortion = np.cos((y*1/25.0))*np.sin((np.exp(x)*1/25.))+5.0 # noisy plume
    distortion = 1.0
    conc = conc * distortion
    conc = (conc > 0)*conc
    # conc = np.array(conc)
    # conc = conc/np.nanmax(conc)

    xp = x #- plume.xs
    yp = y - plume.ys

    x_ = xp*cos(plume.a) - yp*sin(plume.a)
    y_ = xp*sin(plume.a) + yp*cos(plume.a)

    x_ = x_ +plume.xs
    y_ = y_ +plume.ys

    realP = plt.contour (x_,y_,conc, cmap="jet",linewidths=3) #, linewidths=3, cmap=cm.coolwarm)
    plt.colorbar(label='Real plume')

    # if mp is not None:
    #     [x,y] = np.meshgrid(np.linspace(xmin,xmax*2,resolution), np.linspace(ymin,ymax*2,resolution))
    #     Q = mp.Q
    #     Dy = mp.Dy
    #     Dz = mp.Dz
    #     v = mp.v
    #     H = mp.H
    #     xx = x#-mp.xs
    #     yy = y-mp.ys

    #     # plt.axis([-5,world_size+5,-5,world_size+5])
    #     plt.axis([-world_size/2.0,world_size+world_size/2.0,-world_size/2.0,world_size+world_size/2.0])
    #     # plt.axis([world_size/1.5-world_size,world_size+world_size/1.5,world_size/1.5-world_size,world_size+world_size/1.5])
    #     conc=Q/(2*pi*xx*sqrt(Dy*Dz))*np.exp(-H**2*v/(4*Dz*xx))*np.exp(-v*yy**2/(4*Dy*xx))
        
    #     xp = x #- mp.xs
    #     yp = y - mp.ys
    #     x_ = xp*cos(mp.a) - yp*sin(mp.a)
    #     y_ = xp*sin(mp.a) + yp*cos(mp.a)

    #     x_ = x_ +mp.xs;
    #     y_ = y_ +mp.ys;


    #     # normalize conc ---------------------------------------------
    #     # conc = np.array(conc)
    #     # conc = conc/np.nanmax(conc)
    #     plt.contour(x_,y_,conc,cmap='Blues', linewidths=3)
    #     plt.colorbar()

    xs,ys,Q,v,Dy,Dz,H,a = get_states(particles)

    [x,y] = np.meshgrid(np.linspace(xmin,xmax*2,resolution), np.linspace(ymin,ymax*2,resolution))
    Q = sum(Q)/len(Q)
    Dy = sum(Dy)/len(Dy)
    Dz = sum(Dz)/len(Dz)
    v = sum(v)/len(v)
    H = sum(H)/len(H)
    xx = x#-mp.xs
    yy = y-sum(ys)/len(xs)
    
    # plt.axis([-5,world_size+5,-5,world_size+5])

    # plt.axis([-world_size/2.0,world_size+world_size/2.0,-world_size/2.0,world_size+world_size/2.0])

    # plt.axis([world_size/1.5-world_size,world_size+world_size/1.5,world_size/1.5-world_size,world_size+world_size/1.5])
    conc=Q/(2*pi*xx*sqrt(Dy*Dz))*np.exp(-H**2*v/(4*Dz*xx))*np.exp(-v*yy**2/(4*Dy*xx))
    
    # ave_a = (sum(a)%(2.0*pi))/2.0
    y_unit = 0
    x_unit = 0
    for i in range(len(a)):
        y_unit += np.sin(a[i])
        x_unit += np.cos(a[i])
    ave_a = np.arctan2(y_unit,x_unit)%(2.0*pi)  # circular mean for angles

    xp = x #- sum(xs)/len(xs)
    yp = y - sum(ys)/len(ys)

    # x_ = xp*cos(sum(a)/len(a)) - yp*sin(sum(a)/len(a))
    # y_ = xp*sin(sum(a)/len(a)) + yp*cos(sum(a)/len(a))
    
    x_ = xp*cos(ave_a) - yp*sin(ave_a)
    y_ = xp*sin(ave_a) + yp*cos(ave_a)
    
    
    # print sum(a)/len(a)*180.0/pi
    # print sum(xs)/len(xs),sum(ys)/len(ys)

    x_ = x_ +sum(xs)/len(xs);
    y_ = y_ +sum(ys)/len(ys);

    
    # normalize conc ---------------------------------------------
    # conc = np.array(conc)
    # conc = conc/np.nanmax(conc)
    estiP = plt.contour(x_,y_,conc, cmap='Greys',linewidths=3)#markers='-', linewidths=3, cmap=cm.coolwarm)
    plt.colorbar(label='Estimated plume')
    

    # plotting particles
    x,y,Q,v,Dy,Dz,H,a = get_states(particles)
    parDots = plt.scatter(x,y, alpha = .5, s = 25, color='k')
    robots = plt.scatter(plume.x,plume.y,s = 50, color='r', marker='v')
    sourceR = plt.scatter(plume.xs,plume.ys,color='m',s = 150, alpha = 1, marker='*', linewidths=2)
    sourceE = plt.scatter(sum(x)/len(x), sum(y)/len(y),s=150, color='k', marker='x', linewidths=2)
    MLpart = plt.scatter(x[int(N/2.0)],y[int(N/2.0)], alpha = 1, s = 50, color='b')
    # if mp is not None:
        # plt.scatter(mp.xs,mp.ys,s=100, color='b', marker='x', linewidths=2)
    plt.figtext(.16,.875, 'Average Particle Error: ' + str(+eval(plume,particles)), color='b')

    i = 1
    for m in plume.msg:
        plt.figtext(.16,.875-i*.025, 'robot '+str(i-1) + ' state: ' + m, color='b')
        i+=1

    plt.xlabel('xs')
    plt.ylabel('ys')
    plt.legend([realP, estiP, parDots, robots, sourceR, sourceE, MLpart],['Real plume','Estimated plume', 'Particle estimates','robots','Real plume source', 'Estimated plume source', 'most likely particle'])
    
    # -------------------------------------------------------------------------------#
    # plt.figure(2)
    # plt.clf()
    # plt.axis([1.0,3.0,0,2])
    # plt.scatter(Dy,Dz, alpha = .5, s=25, color='k')
    # plt.scatter(plume.Dy,plume.Dz,s=100, color='m', marker='*', alpha=1,linewidths=2)
    # plt.scatter(sum(Dy)/len(Dy), sum(Dz)/len(Dz),s=100, color='g', marker='x', linewidths=2)
    # plt.scatter(Dy[int(N/2.0)],Dz[int(N/2.0)], alpha = 1, s = 50, color='b')
    # # if mp is not None:
    #     # plt.scatter(mp.Dy, mp.Dz, s=100, color='b', marker='x',  linewidths=2)
    # plt.ylabel('Dz')
    # plt.xlabel('Dy')

    # plt.figure(3)
    # plt.clf()
    # plt.axis([0,3.0,.25,3.5])
    # plt.scatter(v,Q, alpha = .5, s=25, color='k')
    # plt.scatter(plume.v,plume.Q,s=100, color='m', marker='*', alpha=1,linewidths=2)
    # plt.scatter(sum(v)/len(v), sum(Q)/len(Q),s=100, color='g', marker='x', linewidths=2)
    # plt.scatter(v[int(N/2.0)],Q[int(N/2.0)], alpha = 1, s = 50, color='b')
    # # if mp is not None:
    #     # plt.scatter(mp.v,mp.Q,s=100, color='b', marker='x', linewidths=2)
    # plt.ylabel('Q')
    # plt.xlabel('v')


    plt.figure(4)
    plt.clf()
    plt.axis([0,13,-.1,2.1*pi])
    plt.scatter(H,a, alpha = .5, s=25, color='k')
    # print '**', plume.H, plume.a
    plt.scatter(plume.H,plume.a,s=100, color='m', marker='*', alpha=1, linewidths=2)
    # plt.scatter(sum(H)/len(H), sum(a)/len(a),s=100, color='g', marker='x', linewidths=2)
    plt.scatter(sum(H)/len(H), ave_a,s=100, color='g', marker='x', linewidths=2)
    plt.scatter(H[int(N/2.0)],a[int(N/2.0)], alpha = 1, s = 50, color='b')
    # if mp is not None:
        # plt.scatter(mp.H,mp.a%(2.0*pi),s=100, color='b', marker='x', linewidths=2)
    plt.ylabel('a')
    plt.xlabel('H')

    # plt.figure(5)
    # plt.clf()
    # # plt.axis([0,N,0,1])

    # # p = range(N)
    # p = range(N)
    # # print len(p), len(w)
    # plt.scatter(range(N), w, s=5, color='k')
    # -------------------------------------------------------------------------------#
    plt.draw()
    # raw_input()
    time.sleep(0.0001)

def collect_p(p,w):
    # this function plots the xs and ys of all the particles within the particledistribution
    from collections import defaultdict
    part = {None: None}
    part = defaultdict(lambda: 0, part)   

    
    for i in range(len(p)):
        part[(p[i].xs, p[i].ys, p[i].Q, p[i].v, p[i].Dy, p[i].Dz, p[i].H)]+=1
    
    # print ''
    # print 'collect', part, len(part)
    print 'distinct particles: ', len(part)-1 # -1 for the {None: None}
    if len(part)-1<=10:
        print 'collect:', part
        find_boundaries(p)
        raw_input()

def find_boundaries(p):
    xs=[]
    ys=[]
    Q=[]
    v=[]
    Dy=[]
    Dz=[]
    H=[]
    for i in range(len(p)):
        xs.append(p[i].xs)
        ys.append(p[i].ys)
        Q.append(p[i].Q)
        v.append(p[i].v)
        Dy.append(p[i].Dy)
        Dz.append(p[i].Dz)
        H.append(p[i].H)

    print max(xs),min(xs)
    print max(ys),min(ys)
    print max(Q),min(Q)
    print max(v),min(v)
    print max(Dy),min(Dy)
    print max(Dz),min(Dz)
    print max(H),min(H)
def print_weights(w,p):
    for i in range(len(w)):
        print w[i],'(', p[i].xs, p[i].ys,')', 'Q',p[i].Q,'H', p[i].H,'Dy', p[i].Dy,'Dz', p[i].Dz

def get_direction(p,myrobot,Z,stdev,stg=''):
    # Z is my observations
    direction = []

    numRobots = myrobot.numRobot

    if stg == 'up':
        angle = pi/2.0
        for i in range(numRobots):
            direction.append(angle)
            myrobot.msg[i]='going up'
    elif stg == 'down':
        angle = 3.0*pi/2.0
        for i in range(numRobots):
            direction.append(angle)
            myrobot.msg[i]='going down'
    elif stg == 'right':
        angle = 0.0
        for i in range(numRobots):
            direction.append(angle)
            myrobot.msg[i]='going right'
    elif stg == 'left':
        angle = pi
        for i in range(numRobots):
            direction.append(angle)
            myrobot.msg[i]='going left'
    elif stg =='ESL':
        # GO TOWARDS THE ESTIMATED SOURCE LOCATION
        xs,ys,Q,v,Dy,Dz,H,a = get_states(p)
        direction = []
        for i in range(numRobots):
            direction.append(atan2(sum(ys)/len(ys)-myrobot.y[i],sum(xs)/len(xs)-myrobot.x[i])) # go in direction of average source location     
            myrobot.msg[i]='Go towards source estimate'
    elif stg =='RBW':
        # random biase walk algorithm
        direction = []
        # print ''
        # raw_input('getting direction')
        # print ''
        # print 'Z:', Z
        for i in range(numRobots):
            if myrobot.lastC[i] != None:
                if Z[i]>myrobot.lastC[i]:
                    genRand = (random.uniform(-5,5)*pi/180.0)%(2.0*pi)
                    direction.append(myrobot.orientation[i]+genRand) # randomly adjust heading by +-5 degrees
                    # print 'greater', genRand*180.0/pi #, Z[i], myrobot.lastC[i], direction[i]*180.0/pi
                    myrobot.msg[i]='Go same direction'
                else:
                    genRand =(random.uniform(0,360)*pi/180.0)%(2.0*pi)
                    direction.append(myrobot.orientation[i]+genRand) # randomly adjust heading by +- 180 degrees
                    # print 'less', genRand*180.0/pi #, Z[i], myrobot.lastC[i], direction[i]*180/pi
                    myrobot.msg[i]='Changing direction'
            else:
                genRand = (random.uniform(0,360)*pi/180.0)%(2.0*pi)
                direction.append(myrobot.orientation[i]+genRand) # randomly adjust heading by +- 180 degrees
                # print 'here', genRand*180.0/pi                
            # reset last Concentration measurement
            myrobot.lastC[i] = Z[i]
    elif stg == 'SMA':
        xs,ys,Q,v,Dy,Dz,H,a = get_states(p)
        ave_xs = sum(xs)/len(xs)
        ave_ys = sum(ys)/len(ys)
        y_unit = 0
        x_unit = 0
        for i in range(len(a)):
            y_unit += np.sin(a[i])
            x_unit += np.cos(a[i])
        a = np.arctan2(y_unit,x_unit)%(2.0*pi)
        
        direction = []

        a_stdev = 0
        for i in range(len(p)):
            # print p[i].a, a, p[i].a-a, (p[i].a-a)**2.0
            a_stdev = a_stdev + (p[i].a-a)**2.0
        a_stdev = sqrt(a_stdev/(N-1))
        # raw_input()
            
        print 'a_stdev:', a_stdev
        for i in range(numRobots):
            if Z[i]>myrobot.conc_thres and a_stdev<1.5: #if i have a concentration measurment head up wind.
                direction.append((a+pi)%(2.0*pi))
                myrobot.msg[i]='Surging'
                # print 'SURGE'
            elif a_stdev<1.5 and Z[i]<myrobot.conc_thres:
                direction.append((a+np.random.choice([-pi/2.0,pi/2.0]))%(2.0*pi))
                myrobot.msg[i]='Casting'
                # print 'CAST'
            else:
                # biased random walk at the beginning
                if myrobot.lastC[i] != None:
                    print 'RANDOM WALK'
                    if Z[i]>myrobot.lastC[i]:
                        genRand = (random.uniform(-5,5)*pi/180.0)%(2.0*pi)
                        direction.append(myrobot.orientation[i]+genRand) # randomly adjust heading by +-5 degrees
                        # print 'greater', genRand*180.0/pi #, Z[i], myrobot.lastC[i], direction[i]*180.0/pi
                        myrobot.msg[i]='Go same direction'
                    else:
                        genRand =(random.uniform(0,360)*pi/180.0)%(2.0*pi)
                        direction.append(myrobot.orientation[i]+genRand) # randomly adjust heading by +- 180 degrees
                        # print 'less', genRand*180.0/pi #, Z[i], myrobot.lastC[i], direction[i]*180/pi
                        myrobot.msg[i]='Changing direction'
                else:
                    genRand = (random.uniform(0,360)*pi/180.0)%(2.0*pi)
                    direction.append(myrobot.orientation[i]+genRand) # randomly adjust heading by +- 180 degrees
                    # print 'here', gaenRand*180.0/pi                
                    # reset last Concentration measurement
                myrobot.lastC[i] = Z[i]

    else:
        direction.append(pi/2.0)

    print 'directon vector', direction

    if stdev<20 and stg != 'ESL':
        print 'GOING TOWARDS THE SOURCE'
        xs,ys,Q,v,Dy,Dz,H,a = get_states(p)
        direction = []
        for i in range(numRobots):
            direction.append(atan2(sum(ys)/len(ys)-myrobot.y[i],sum(xs)/len(xs)-myrobot.x[i])) # go in direction of average source location     
            myrobot.msg[i]='Go towards source estimate'            


    # angle = [(b*180.0/pi)%360 for b in direction]
    # print 'angle set', angle
    # print 'est. wind:', a*180.0/pi
    
    # print ''
    # raw_input('getting direction')
    # print ''

    
    return direction

myrobot = particlePlume(realPlume=1, numRobot=4)
predicted_noise = 0.003
real_noise = 0.001
default = 'SMA' # set the direction algorthm
noise_ratio = predicted_noise/real_noise
# must have noise_ratio>0 or bad estimate 
print 'noise_ratio:', noise_ratio
myrobot.set_noise(0.5,0.5,real_noise)

debug = False

fig2=plt.figure()
plt.ion()

# from mpl_toolkits.mplot3d import Axes3D
N = 1000
Nt = N*.6
T = 1000
p = []

# initalizae all particles states randomly and set noise
for i in range(N):
    r = particlePlume(realPlume=0)
    r.set_noise(0.5,0.5,predicted_noise) # forward noise, turning noise, sensor noise
    p.append(r)

w = range(N) # initalized weights
plot_PF(p,myrobot,w=w)
raw_input()
# raw_input('init')
mp = None
dflag = 0
cflag = 0
Z = myrobot.sense(myrobot)
stdev = 10000000
for t in range(T):
    # print 'sensed before', Z
    # print myrobot.x, myrobot.y
    if mp is not None: # init special case handling
        direction = get_direction(p,myrobot,Z,stdev,default)
    else:
        direction = get_direction(p,myrobot,Z,stdev,default)

    length = 5.0 # distance my robot moves each estimation cycle
    # move robot
    myrobot = myrobot.move(direction,length,myrobot=1)

    # get new observation for each robot
    Z = myrobot.sense(myrobot)
    print 'sensed', Z
# ----------- 

    # prediction move particles through gaussian distribution 
    p2 = []
    if w != range(N):
        for i in range(N):
            p2.append(p[i].move_particle(w[i], dflag))#p[i],w[i]))
        p = p2
    # plot_PF(p,myrobot, mp, w)
    
    # update weights
    for i in range(N):
        if debug:
            print 'weight update:', 'particle', i
        w[i] = p[i].measurement_prob(Z,myrobot) # i normalize weightings every cycle
    if debug:
        raw_input()
   
    # find N_eff
    p3 = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    sw = sum(w)
    N_eff = 0
    if debug:
        print 'weightings', w
    
    xs,ys,Q,v,Dy,Dz,H,a = get_states(p)
    
    ave_xs = sum(xs)/len(xs)
    ave_ys = sum(ys)/len(ys)
    ave_Q = sum(Q)/len(Q)
    ave_v = sum(v)/len(v)
    ave_Dy = sum(Dy)/len(Dy)
    ave_Dz = sum(Dz)/len(Dz)
    ave_H = sum(H)/len(H)
    y_unit = 0
    x_unit = 0
    for i in range(len(a)):
        y_unit += np.sin(a[i])
        x_unit += np.cos(a[i])
    ave_a = np.arctan2(y_unit,x_unit)%(2.0*pi)

    stdev = 0
    for i in range(N):
        # need to have a standard vector for parameters so I can change them with ease.

        stdev = stdev + (p[i].xs-ave_xs)**2 + (p[i].ys-ave_ys)**2 + (p[i].Q-ave_Q)**2 + (p[i].v-ave_v)**2 + (p[i].Dy-ave_Dy)**2 + (p[i].Dz-ave_Dz)**2 + (p[i].H-ave_H)**2 + (p[i].a-ave_a)**2

        if w[i] == mw:
            mp = p[i]
            # print i, p[i], w[i]
        if sw != 0:
            w[i] = w[i]/sw
        
        if w[i] != 0:
            # print 'w:',w[i]
            N_eff += (w[i]**2)

    stdev = sqrt(stdev/(N-1))
    
    print 'stdev:', stdev
    
    # WHY IS DEGENERACY NOT HAPPENING WHEN i HAVE VERY FEW PARTICLES????
    if debug:
        print 'normalize w', w

    if N_eff != 0:
        N_eff = 1/N_eff
        # print 'N_eff:', N_eff
        if N_eff <= 1.5: # made the degenerative case larger 
            dflag = 1
            print 'N_eff = degeneracy'
        else:
            dflag = 0
    else:
        dflag = 1
        print 'N_eff = degeneracy'

    # print 'dflag', dflag

    # print 'maxw', max(w)#, 'sumw',sum(w)

    # resample
    # wheel resmapling by thrun's class
    # if True and any(cns>=myrobot.conc_thres for cns in Z):
    #     cflag = 1
    #     print 'resampled'
    #     mw = max(w)
    #     for i in range(N):
    #         beta += random.random() * 2.0 * mw
    #         while beta > w[index]:
    #             beta -= w[index]
    #             index = (index + 1) % N
    #         p3.append(p[index])
    #     p = p3


    # systematic like resampling
    # if True and any(cns>=myrobot.conc_thres for cns in Z):
    #     cflag = 1
    #     p3 = [0]*N
    #     index = 0
    #     w_sum = sum(w)
    #     step = w_sum/N
    #     # beta = (w_sum*w_sum)%step
    #     beta = random.random()*step
    #     # beta = random.random()*1.0/N
    #     for i in range(N):
    #         while beta > w[index]:
    #             beta -= w[index]
    #             index = (index+1)%N
    #         beta+=step
    #         p3[i] = p[index]
    #     p = p3
    
    # print w

    if sum(w) != 0:#N_eff<Nt:# and any(cns>=myrobot.conc_thres for cns in Z):
        # print 'resampled'
        cflag = 1
        p3 = [0]*N
        index = 0
        c = w[index]
        r = random.random()*1.0/N
        for i in range(N):
            u = r + (i-1)/(N*1.0)
            while u > c:
                index = index+1
                c = c + w[index]
            p3[i] = p[index]
        p = p3
    # print w
    # raw_input()
    plot_PF(p,myrobot, mp,w)



