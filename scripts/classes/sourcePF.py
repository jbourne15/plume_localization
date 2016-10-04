#!/usr/bin/env python
from math import *
import random
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import time
import matplotlib
from matplotlib import cm
from munkres import Munkres, print_matrix

world_size = 200.0

class particlePlume:
    def __init__(self,realPlume = 0, numRobot=1):
        
        if realPlume: # if this is true, this initalizes my location of the real plume xs,ys and will also
                      # create my numRobot robots
            # location of my real plumen
            # these are my state variables
            self.ys = 155
            self.xs = 100
            # self.Q  = 1.9
            self.Q = 8.5
            self.v  = 2.75
            self.Dy = 6.25
            self.Dz = 2.0
            self.H  = 5
            # self.a = 1.6*pi
            # self.a = 297*pi/180.0
            self.a = 270*pi/180


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
            
            self.pathx = np.zeros(self.numRobot)
            self.pathy = np.zeros(self.numRobot)

            # self.conc_thres = 0.001
            self.conc_thres = 0.0006
            self.clusteredData = None
            self.old_clusteredData = None
            self.c = np.ones((self.numRobot,8))*100000 # 8 states old  clustered centroid 
            

            for i in range(self.numRobot):
                # self.x.append(random.random()*world_size)
                # self.y.append(random.random()*world_size)
                self.y.append(0)
                self.x.append(170.0-i*40.0)
                # if i<6:
                    # self.y.append(160-i*20.0)
                    # self.x.append(100)
                # elif i>=6:
                    # self.y.append(160-(i-6)*20.0)
                    # self.x.append(95)

                # self.orientation.append(random.random() * 2.0 * pi)
                self.lastC.append(None)
                # print 'setting lastC to None'
                self.orientation.append(pi/2.0)
                self.msg.append('init')
            # print 'orientation', self.orientation
        else:            # this is the simulated particles with random.random() 'guessing' the states
                         # notice that some of my states are given to me, but I could make them hidden
            
            self.cluster = None
            # state space
            self.X_SS  = [0, world_size]
            self.Y_SS = [0, world_size]
            self.Q_SS = [.5, 8.5]
            self.V_SS = [.25, 5.25]
            self.Dy_SS = [1.5, 6.5]
            self.Dz_SS = [.5, 5.5]
            self.H_SS  = [2, 12]
            self.A_SS = [0, 2.0*pi]

            # unknown variables or hidden states
            self.xs = random.random()*(self.X_SS[1]-self.X_SS[0]) + self.X_SS[0] # randomly set particle x location
            self.ys = random.random()*(self.Y_SS[1]-self.Y_SS[0]) + self.Y_SS[0]
            self.Q  = random.random()*(self.Q_SS[1]-self.Q_SS[0]) + self.Q_SS[0]
            self.v  = random.random()*(self.V_SS[1]-self.V_SS[0]) + self.V_SS[0]
            self.Dy = random.random()*(self.Dy_SS[1]-self.Dy_SS[0]) + self.Dy_SS[0]
            self.Dz = random.random()*(self.Dz_SS[1]-self.Dz_SS[0]) + self.Dz_SS[0]
            self.H  = random.random()*(self.H_SS[1]-self.H_SS[0]) + self.H_SS[0]
            self.a  = random.random()*(self.A_SS[1]-self.A_SS[0]) + self.A_SS[0]

        
        # init noise parameter, will change later through method
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;
    
    def set(self, new_x, new_y, new_orientation,lastC, msg,pathx,pathy, clusteredD = None, c=None):#lastC
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
            self.pathx = pathx
            self.pathy = pathy
            self.clusteredData = clusteredD
            self.c = c
    
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
        sig_QDyDz = .15
        # sig_QDyDz = .005
        # sig_H = .1
        sig_H = .25
        # sig_v  = .025
        sig_v  = .15
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


        while True:
            res.xs  = random.gauss(self.xs,sig_xy)
            if res.X_SS[0]<res.xs and res.X_SS[1]>res.xs:
                break
        while True:
            res.ys  = random.gauss(self.ys,sig_xy)
            if res.Y_SS[0]<res.ys and res.Y_SS[1]>res.ys:
                break
        while True:
            res.Q = random.gauss(self.Q,sig_QDyDz)
            if res.Q_SS[0]<res.Q and res.Q_SS[1]>res.Q:
                break
        while True:
            res.v  = random.gauss(self.v,sig_v)
            if res.V_SS[0]<res.v and res.V_SS[1]>res.v:
                break
        while True:
            res.Dy  = random.gauss(self.Dy,sig_QDyDz)
            if res.Dy_SS[0]<res.Dy and res.Dy_SS[1]>res.Dy:
                break
        while True:
            res.Dz  = random.gauss(self.Dz,sig_QDyDz)
            if res.Dz_SS[0]<res.Dz and res.Dz_SS[1]>res.Dz:
                break
        while True:
            res.H  = random.gauss(self.H,sig_H)
            if res.H_SS[0]<res.H and res.H_SS[1]>res.H:
                break
        while True:
            res.a = random.gauss(self.a,sig_a)#%(2.0*pi)
            # if res.a>2*pi:
                # res.a = res.a%(2.0*pi)
            if res.A_SS[0]<res.a and res.A_SS[1]>res.a:
                break
            
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        res.cluster = self.cluster
        return res
        
    def move(self, turn, forward, myrobot=0):
                                
        # add in a check to see if I am already close to the desired point, then don't move ------------------------------------------------------------------<-----------------------------<--------------------<-
        
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
            res.set(x, y, orientation, myrobot.msg, myrobot.pathx, myrobot.pathy) # set new particle states
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
            res.set(x,y,orientation,self.lastC,self.msg,self.pathx, self.pathy, self.clusteredData, self.c)
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

            
            # replaced with 
            # if default == 'SMA' or default == 'RBW':
            #     conc[i] = conc[i] * distortion + noise
            #     # print 'here'
            # else:
            #     if conc[i]*distortion+noise < self.conc_thres: # I can only sense after a certain threshold 
            #         conc[i] = 0
            #         # print 'or herer'
            #     else:
            #         # distortion = 1.0
            #         conc[i] = conc[i] * distortion + noise
            #         # print 'or herer'

            if conc[i]*distortion+noise < self.conc_thres: # I can only sense after a certain threshold 
                conc[i] = 0
                # print 'or herer'
            else:
                # distortion = 1.0
                conc[i] = conc[i] * distortion + noise
                # print 'or herer'

            
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
    sa = np.arctan2(y_unit,x_unit)%2.0*pi  # circular mean for angles


    dx = (sxs - r.xs + (world_size/2.0)) % world_size - (world_size/2.0)
    dy = (sys - r.ys + (world_size/2.0)) % world_size - (world_size/2.0)
    dq = (sQ  - r.Q)
    dv = (sv  - r.v)
    dDy = (sDy - r.Dy)
    dDz = (sDz - r.Dz)
    dH = (sH - r.H)
    da = (sa+pi - (r.a%(2.0*pi)))

    # print 'angle difference', da
    # print sa+pi, (r.a%(2.0*pi))
    
    err = np.array((sqrt(dq**2.0), sqrt(dv**2.0), sqrt(dDy**2.0 + dDz**2.0), sqrt(dH**2.0), sqrt(da**2.0), sqrt(dx**2.0 + dy**2.0)))
    # row=np.array((ave_xs, ave_ys, ave_Q, ave_v, ave_Dy, ave_Dz, ave_H, ave_a))

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

def plot_error(plume, p, error, robot_data, t, param_stdev):
    
    plt.figure('errorfig',figsize=(8,12))
    plt.clf()
    
    # if default == 'RBW':
        
    #     bioerror = plt.scatter(range(t),error,s=25, color='r', marker='.')

    #     plt.xlabel('Iteration number',fontsize=18)
    #     plt.ylabel('Average positional error',fontsize=20)
    #     plt.title('Average positional error vs. Iteration number',fontsize=20)
    #     plt.legend([bioerror],['Average positional error'],fontsize=20)
        
    #     plt.draw
    #     time.sleep(0.0001)
        
    #     if t%10 == 0 or t==1:
    #         if t==1:
    #             savestg = default+'_error_at_t_'+str(0)+extension
    #             raw_input()
    #         else:
    #             savestg = default+'_error_at_t_'+str(t)+extension
    #         plt.savefig(savestg, transparent = True)


    # else:
    #     pferror = plt.scatter(range(t),error,s = 25, color='r', marker='.')
        
    #     plt.xlabel('Iteration number',fontsize=20)
    #     plt.ylabel('Average error',fontsize=20)
    #     plt.title('Average error vs. Iteration number',fontsize=20)
    #     plt.legend([pferror],['Average particle estimate error'],fontsize=20)
        
    #     plt.draw
    #     time.sleep(0.0001)
        
    #     if t%10 == 0 or t==1:
    #         if t==1:
    #             savestg = default+'_error_at_t_'+str(0)+extension
    #             raw_input()
    #         else:
    #             savestg = default+'_error_at_t_'+str(t)+extension
    #         plt.savefig(savestg, transparent = True)
    

    # err = (sqrt(dx*dx + dy*dy), sqrt(dq*dq), sqrt(dv*dv), sqrt(dDy*dDy + dDz*dDz), sqrt(dH*dH), sqrt(da*da))
        
    labels = ('Average release rate error', 'Average velocity error', 'Average diffusivity error', 'Average stack height error', 'Average upwind angle error','Average positional error')
    units = ('$(m)$', '$(\frac{kg}{m^3})', '$(\frac{m}{s})', '$(\frac{m^2}{s})', '$(m)$', '$radians$')
    markers= ('*','|', '.', '_', 'x', '+')
    # print t, range(t), error
    # print error.shape[1]
    # raw_input()
    


    if all(error[0,:]==0) and t != 0:
        error = error[1:error.shape[0],:]


    if all(param_stdev[0,:]==0) and t != 0:
        param_stdev = param_stdev[1:param_stdev.shape[0],:]


    # print error 
    # print param_stdev
    # print error.shape
    # print param_stdev.shape
    # print range(t)
    # raw_input()

    for i in range(error.shape[1]):
        # print 'i', i
        plt.subplot(6,1,i+1)
        plt.title(labels[i]+' vs. time step')

        if np.max(param_stdev[:,i])<np.max(error[:,i]):
            plt.axis([-1,T*1.1,-1,np.max(error[:,i])*1.1])
        else:
                  plt.axis([-1,T*1.1,-1,np.max(param_stdev[:,i])*1.1])
    

        # print units[i]
        if i == 5:
            # plt.ylabel(r'$(m)$',fontsize=12)
            plt.ylabel(r'$(m)$', fontsize=20)
        elif i==0:
            plt.ylabel(r'$(\frac{kg}{s})$', fontsize=20)
        elif i==1:
            plt.ylabel(r'$(\frac{m}{s})$',fontsize=20)
        elif i==2:
            plt.ylabel(r'$(\frac{m^2}{s})$',fontsize=20)
        elif i==3:
            plt.ylabel(r'$(m)$',fontsize=20)
        elif i==4:
            plt.ylabel(r'$(radians)$',fontsize=20)
        # print i,labels[i], np.max(error[:,i])
        
        if t == 0:
            pferror = plt.scatter(0,error[:,i],s = 15, color='r', marker='.', label=labels[i])
            stdev_param = plt.scatter(0,param_stdev[:,i], s=15, color='b', marker='+', label='Standard deviation')
        else:
            pferror = plt.scatter(range(error.shape[0]),error[:,i],s = 15, color='r', marker='.', label=labels[i])
            stdev_param = plt.scatter(range(param_stdev.shape[0]),param_stdev[:,i], s=15, color='b', marker='+', label='Standard deviation')
        

        # pferror = plt.scatter(range(t+1),error[:,i],s = 15, color='r', marker='.', label=labels[i])
        # stdev_param = plt.scatter(range(t+1),param_stdev[:,i], s=15, color='b', marker='+', label='Standard deviation')

        # q, v, dydz, h, a, xsys

    plt.xlabel('Time step',fontsize=20)
    plt.tight_layout()
    


    # plt.title('Average error vs. Iteration number',fontsize=20)

    # plt.legend([pferror],['Average particle estimate error'],fontsize=20)


    # plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           # ncol=2, mode="expand", borderaxespad=0.)

    
    plt.draw
    time.sleep(0.0001)
    
    # raw_input()

    if t%10 == 0 or t==1:
        if t==1:
            savestg = default+'_error_at_t_'+str(0)+extension
            # raw_input()
        else:
            savestg = default+'_error_at_t_'+str(t)+extension
        # plt.savefig(savestg, transparent = True)
        
    if t==100:
        x,y,Q,v,Dy,Dz,H,a = get_states(p)

        y_unit = 0
        x_unit = 0
        for i in range(len(a)):
            y_unit += np.sin(a[i])
            x_unit += np.cos(a[i])
        ave_a = np.arctan2(y_unit,x_unit)%(2.0*pi)  # circular mean for angles

        print 'aveXs: ', sum(x)/len(x), 'aveYs: ',sum(y)/len(y), 'aveQ: ',sum(Q)/len(Q), 'aveV: ',sum(v)/len(v), 'aveDy: ',sum(Dy)/len(Dy), 'aveDz: ',sum(Dz)/len(Dz), 'aveH: ',sum(H)/len(H), 'ave_a: ',ave_a
        print 'xs', myrobot.xs, 'ys', myrobot.ys, 'Q', myrobot.Q, 'v',myrobot.v, 'Dy', myrobot.Dy, 'Dz', myrobot.Dz, 'H',myrobot.H, 'a', myrobot.a


def plot_PF(particles,plume,t, mp=None, w=None):
    # right now I am replotting the stationary plume this is slow
    
    
    plt.figure('simfig',figsize=(18,11))
    plt.clf()
    plt.axis([-world_size/10.0,world_size+world_size/10.0,-world_size/10.0,world_size+world_size/10.0])

    xmin = 0.000000000
    xmax = world_size*2
    ymin = 0.000000000
    ymax = world_size*2
    resolution = 100


    #------------------------ estimated plume
    xs,ys,Q,v,Dy,Dz,H,a = get_states(particles)

    [x,y] = np.meshgrid(np.linspace(xmin,xmax*2,resolution), np.linspace(ymin,ymax*2,resolution))
    Q = sum(Q)/len(Q)
    Dy = sum(Dy)/len(Dy)
    Dz = sum(Dz)/len(Dz)
    v = sum(v)/len(v)
    H = sum(H)/len(H)
    xx = x#-mp.xs
    yy = y-sum(ys)/len(xs)

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


    estiP = plt.contour(x_,y_,conc, cmap='jet',linewidths=3, linestyles='dashed', label='Estimated plume (kg/m^3)')#markers='-', linewidths=3, cmap=cm.coolwarm)
    cbar = plt.colorbar()
    cbar.set_label('Estimated plume '+ r'$(\frac{kg}{m^3})$', size=20)
    #------------------------ estimated plume
    
    [x,y] = np.meshgrid(np.linspace(xmin,xmax,resolution), np.linspace(ymin,ymax,resolution))

    Q = plume.Q
    Dy = plume.Dy
    Dz = plume.Dz
    v = plume.v
    H = plume.H
    
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
    
    realP = plt.contour (x_,y_,conc, cmap="jet",linewidths=5, label='Real plume (kg/m^3)') #, linewidths=3, cmap=cm.coolwarm)
    cbar = plt.colorbar()
    cbar.set_label('Real plume '+ r'$(\frac{kg}{m^3})$', size=20)
        
    x,y,Q,v,Dy,Dz,H,a = get_states(particles)
    markersize=15
    # for p in particles:
    #     # # print p.cluster
    #     # print 'hi', p.cluster
    #     if p.cluster == 0:
    #         plt.scatter(p.xs,p.ys, alpha = .5, s = 5, color='r', label='Particle estimates')
    #     elif p.cluster == 1:
    #         plt.scatter(p.xs,p.ys, alpha = .5, s = 5, color='g', label='Particle estimates')
    #     elif p.cluster == 2:
    #         plt.scatter(p.xs,p.ys, alpha = .5, s = 5, color='b', label='Particle estimates')
    #     elif p.cluster == None:
    #         break

    
    # for p in particles:
    #     print p.cluster

    parDots = plt.scatter(x,y, alpha = .5, s = 5, color='k', label='Particle estimates')
    robots = plt.plot(plume.x,plume.y, 'rv',ms = markersize, label='Robots')
    sourceR = plt.plot(plume.xs,plume.ys,'m*',ms = markersize+10, alpha = 1, label='Real plume source')
    sourceE = plt.plot(sum(x)/len(x), sum(y)/len(y),'c*', ms=markersize+10,  label='Estimated plume source')
    # MLpart = plt.scatter(x[int(N/2.0)],y[int(N/2.0)], alpha = 1, s = 50, color='b') # most likely particle

    # store_data = np.vstack([store_data, row])
    rowx = []
    rowy = []
    for i in range(myrobot.numRobot):
        rowx.append(myrobot.x[i])
        rowy.append(myrobot.y[i])

    # print myrobot.pathx
    # print 'addingx', rowx
    # print 'addingy', rowy

    myrobot.pathx = np.vstack([myrobot.pathx, rowx])
    myrobot.pathy = np.vstack([myrobot.pathy, rowy])

    for i in range(myrobot.numRobot):
        if i == 0:
            path = plt.plot(myrobot.pathx[1:myrobot.pathx.shape[0],i],myrobot.pathy[1:myrobot.pathy.shape[0],i], label='Robot path',linestyle='-', marker='.')
        else:
            plt.plot(myrobot.pathx[1:myrobot.pathx.shape[0],i],myrobot.pathy[1:myrobot.pathy.shape[0],i], label='Robot path',linestyle='-', marker='.')


    # plotting particles
    # print plume.clusteredData
    if plume.clusteredData != None:
        # print 'cluster:, xs, ys, Q, v, Dy, Dz, H, a'
        for i in range(plume.numRobot):
            plt.scatter(plume.clusteredData[i,0],plume.clusteredData[i,1],s=100, color='r', marker='+', linewidths=2)
            
            [x,y] = np.meshgrid(np.linspace(xmin,xmax*2,resolution), np.linspace(ymin,ymax*2,resolution))
            xx = x#-mp.xs
            yy = y-plume.clusteredData[i,1]

            # clusterData: xs, ys, Q, v, Dy, Dz, H, ap
            Q = plume.clusteredData[i, 2]
            v = plume.clusteredData[i, 3]
            Dy = plume.clusteredData[i, 4]
            Dz = plume.clusteredData[i, 5]
            H = plume.clusteredData[i, 6]
            a = plume.clusteredData[i, 7]
            
            x_cl = myrobot.clusteredData[i,0]
            y_cl = myrobot.clusteredData[i,1]
                         # ys+sin(a)*H**2.0*v/(4*Dz)
            y_hc = y_cl + sin(a)*H**2.0*v/(4*Dz)
                       # xs+cos(a)*H**2.0*v/(4*Dz)
            x_hc = x_cl + cos(a)*H**2.0*v/(4*Dz)
            
            plt.scatter(x_hc,y_hc,s=50, color='m', marker='x', linewidths=2)

            # print 'cluster:',plume.clusteredData[i,0], plume.clusteredData[i,1], Q, v, Dy, Dz, H, a
            
            conc=Q/(2*pi*xx*sqrt(Dy*Dz))*np.exp(-H**2*v/(4*Dz*xx))*np.exp(-v*yy**2/(4*Dy*xx))
            
            xp = x #- sum(xs)/len(xs)
            yp = y - plume.clusteredData[i,1]
            
            x_ = xp*cos(a) - yp*sin(a)
            y_ = xp*sin(a) + yp*cos(a)
            
            x_ = x_ +plume.clusteredData[i,0]
            y_ = y_ +plume.clusteredData[i,1]
        
            
            clusteredEsti = plt.contour(x_,y_,conc, cmap='Greys',linewidths=1)
        # raw_input()
            
    plt.xlabel(r'$x_s$ '+ r'$(m)$', fontsize=20)
    plt.ylabel(r'$y_s$ '+ r'$(m)$',fontsize=20)
    # plt.legend([realP, estiP, parDots, robots, sourceR, sourceE],['Real plume (kg/m^3)','Estimated plume (kg/m^3)', 'Particle estimates','Robots','Real plume source', 'Estimated plume source'],fontsize=20)

    # plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
               # ncol=2, mode="expand", borderaxespad=0., fontsize=10)

    # plt.legend([realP, robots, sourceR],['Real plume (kg/m^3)','robots','Real plume source'],fontsize=20)
                

    # -------------------------------------------------------------------------------#
    xs,ys,Q,v,Dy,Dz,H,a = get_states(particles)

    plt.figure('other_parameters',figsize=(6,11))
    plt.clf()
    plt.axis([particles[1].Dy_SS[0]*.8,particles[1].Dy_SS[1]*1.05,particles[1].Dz_SS[0]*.8,particles[1].Dz_SS[1]*1.05])
    
    # Cave_Dy = np.zeros(myrobot.numRobot)
    # Cave_Dz = np.zeros(myrobot.numRobot)
    # Cave_v  = np.zeros(myrobot.numRobot)
    # Cave_Q  = np.zeros(myrobot.numRobot)
    # Cave_H  = np.zeros(myrobot.numRobot)
    # Cave_a  = np.zeros(myrobot.numRobot)
    # y_unit = np.zeros(myrobot.numRobot)
    # x_unit = np.zeros(myrobot.numRobot)
    # num_C_particles = np.zeros(myrobot.numRobot)

        # if p.cluster == 0:
        #     plt.scatter(p.xs,p.ys, alpha = .5, s = 5, color='r', label='Particle estimates')
        # elif p.cluster == 1:
        #     plt.scatter(p.xs,p.ys, alpha = .5, s = 5, color='g', label='Particle estimates')
        # elif p.cluster == 2:
        #     plt.scatter(p.xs,p.ys, alpha = .5, s = 5, color='b', label='Particle estimates')
        # elif p.cluster == None:
        #     break

    #slow
    # for p in particles:
    #     if p.cluster == 0:
    #         plt.subplot(3,1,1)
    #         plt.scatter(p.Dy,p.Dz, alpha = .5, s = 5, color='r', label='Particle estimates')
    #         plt.subplot(3,1,2)
    #         plt.scatter(p.v,p.Q, alpha = .5, s = 5, color='r', label='Particle estimates')
    #         plt.subplot(3,1,3)
    #         plt.scatter(p.H,p.a, alpha = .5, s = 5, color='r', label='Particle estimates')

    #     elif p.cluster == 1:
    #         plt.subplot(3,1,1)
    #         plt.scatter(p.Dy,p.Dz, alpha = .5, s = 5, color='g', label='Particle estimates')
    #         plt.subplot(3,1,2)
    #         plt.scatter(p.v,p.Q, alpha = .5, s = 5, color='g', label='Particle estimates')
    #         plt.subplot(3,1,3)
    #         plt.scatter(p.H,p.a, alpha = .5, s = 5, color='g', label='Particle estimates')
    #     elif p.cluster == 2:
    #         plt.subplot(3,1,1)
    #         plt.scatter(p.Dy,p.Dz, alpha = .5, s = 5, color='b', label='Particle estimates')
    #         plt.subplot(3,1,2)
    #         plt.scatter(p.v,p.Q, alpha = .5, s = 5, color='b', label='Particle estimates')
    #         plt.subplot(3,1,3)
    #         plt.scatter(p.H,p.a, alpha = .5, s = 5, color='b', label='Particle estimates')
    #     elif p.cluster == None:
    #         break

        # num_C_particles[p.cluster]+=1
        # Cave_Dy[p.cluster]+=p.Dy
        # Cave_Dz[p.cluster]+=p.Dz
        # Cave_v[p.cluster] +=p.v
        # Cave_Q[p.cluster] +=p.Q
        # Cave_H[p.cluster] +=p.H
        # y_unit[p.cluster] += np.sin(p.a)
        # x_unit[p.cluster] += np.cos(p.a)
    
    # for i in range(myrobot.numRobot):
        # Cave_a[i] = np.arctan2(y_unit[i],x_unit[i])%(2.0*pi)  # circular mean for angles

    # Cave_Q = np.divide(Cave_Q,num_C_particles)
    # Cave_v = np.divide(Cave_v,num_C_particles)
    # Cave_Dy = np.divide(Cave_Dy,num_C_particles)
    # Cave_Dz = np.divide(Cave_Dz,num_C_particles)
    # Cave_H = np.divide(Cave_H,num_C_particles)
    # Cave_Q = np.divide(Cave_a,num_C_particles)

    # print 'Q, v, Dy, Dz, H, a', num_C_particles
    # print Cave_Q, Cave_v, Cave_Dy, Cave_Dz, Cave_H, Cave_a

    #----------------------------#
    plt.subplot(3,1,1)
    if plume.clusteredData != None:
        for i in range(plume.numRobot):
            # clusterData: xs, ys, Q, v, Dy, Dz, H, a
            plt.scatter(plume.clusteredData[i,4],plume.clusteredData[i,5],s=100, color='r', marker='+', linewidths=2)

    plt.scatter(Dy,Dz, alpha = .5, s=5, color='k')
    plt.scatter(plume.Dy,plume.Dz,s=100, color='m', marker='*', alpha=1,linewidths=2)
    plt.scatter(sum(Dy)/len(Dy), sum(Dz)/len(Dz),s=100, color='c', marker='*', linewidths=2)
    # plt.scatter(Dy[int(N/2.0)],Dz[int(N/2.0)], alpha = 1, s = 50, color='b')
    plt.ylabel(r'$D_z$',fontsize=18)
    plt.xlabel(r'$D_y$',fontsize=18)
    #----------------------------#
    plt.subplot(3,1,2)
    plt.axis([particles[1].V_SS[0]*.8,particles[1].V_SS[1]*1.05,particles[1].Q_SS[0]*.8,particles[1].Q_SS[1]*1.05])
    if plume.clusteredData != None:
        for i in range(plume.numRobot):
            # clusterData: xs, ys, Q, v, Dy, Dz, H, a
            plt.scatter(plume.clusteredData[i,3],plume.clusteredData[i,2],s=100, color='r', marker='+', linewidths=2)

    plt.scatter(v,Q, alpha = .5, s=5, color='k')
    plt.scatter(plume.v,plume.Q,s=100, color='m', marker='*', alpha=1,linewidths=2)
    plt.scatter(sum(v)/len(v), sum(Q)/len(Q),s=100, color='c', marker='*', linewidths=2)
    # plt.scatter(v[int(N/2.0)],Q[int(N/2.0)], alpha = 1, s = 50, color='b')
    plt.ylabel(r'$Q$',fontsize=18)
    plt.xlabel(r'$v$',fontsize=18)
    #----------------------------#
    plt.subplot(3,1,3)
    plt.axis([particles[1].H_SS[0]*.8,particles[1].H_SS[1]*1.05,particles[1].A_SS[0],particles[1].A_SS[1]])

    if plume.clusteredData != None:
        for i in range(plume.numRobot):
            # clusterData: xs, ys, Q, v, Dy, Dz, H, a
            plt.scatter(plume.clusteredData[i,6],plume.clusteredData[i,7],s=100, color='r', marker='+', linewidths=2)

    plt.scatter(H,a, alpha = .5, s=5, color='k')
    plt.scatter(plume.H,plume.a,s=100, color='m', marker='*', alpha=1, linewidths=2)
    plt.scatter(sum(H)/len(H), ave_a,s=100, color='c', marker='*', linewidths=2)
    # plt.scatter(H[int(N/2.0)],a[int(N/2.0)], alpha = 1, s = 50, color='b')
    plt.ylabel(r'$\theta$',fontsize=18)
    plt.xlabel(r'$h$',fontsize=18)

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
    # raw_input()
    if t==0:
        raw_input()
    if t%10 == 0:
        savestg = default+'_out_at_t_'+str(t)+extension
        # plt.savefig(savestg, transparent = True)

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

def choose_CC(p, numRobots,c):
    # print 'c: ',c
    # print np.all(c == 100000)
    if np.all(c == 100000):
        c = np.zeros((numRobots,8)) # 8 states
        for i in range(numRobots):
            temp = p[int(random.random() * len(p))]
            temp = np.array([temp.xs, temp.ys, temp.Q, temp.Dy, temp.Dz, temp.v, temp.H, temp.a])
            c[i,:] = temp # randomly choose cluster centroids
        # print 'random c'
        return c
    else:
        # print 'using last c'
        return c

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
        foundC = np.zeros(myrobot.numRobot)
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
            if Z[i]>myrobot.conc_thres and (a_stdev<2.0 or stdev<45): #if i have a concentration measurment head up wind.
                direction.append((a+pi)%(2.0*pi))
                myrobot.msg[i]='SURGING'
                print 'SURGE'
            elif a_stdev<2.0 and Z[i]<myrobot.conc_thres and stdev<45:
                # print Z[i]<myrobot.conc_thres, Z[i], myrobot.conc_thres
                # raw_input()
                print 'observations:', Z
                direction.append((a+np.random.choice([-pi/2.0,pi/2.0]))%(2.0*pi))
                myrobot.msg[i]='CASTING'
                print 'CAST'
            else:
                # biased random walk at the beginning
                if myrobot.lastC[i] != None:
                    # print 'RANDOM WALK'
                    if Z[i]>myrobot.lastC[i]:
                        genRand = (random.uniform(-5,5)*pi/180.0)%(2.0*pi)
                        direction.append(myrobot.orientation[i]+genRand) # randomly adjust heading by +-5 degrees
                        # print 'greater', genRand*180.0/pi #, Z[i], myrobot.lastC[i], direction[i]*180.0/pi
                        myrobot.msg[i]='Going same direction'
                        print 'Going same direction'
                    else:
                        genRand =(random.uniform(0,360)*pi/180.0)%(2.0*pi)
                        direction.append(myrobot.orientation[i]+genRand) # randomly adjust heading by +- 180 degrees
                        # print 'less', genRand*180.0/pi #, Z[i], myrobot.lastC[i], direction[i]*180/pi
                        myrobot.msg[i]='Changing direction'
                        print 'Changning direction'
                else:
                    genRand = (random.uniform(0,360)*pi/180.0)%(2.0*pi)
                    direction.append(myrobot.orientation[i]+genRand) # randomly adjust heading by +- 180 degrees
                    # print 'here', gaenRand*180.0/pi                
                    # reset last Concentration measurement
                myrobot.lastC[i] = Z[i]

    elif stg == 'clustering':
        xs,ys,Q,v,Dy,Dz,H,a = get_states(p)
        myrobot.c = np.ones((numRobots,8))*100000 # uncomment to initialize cluster centroids randomly every time step
        # c = np.ones((numRobots,8))*100000 
        # BUG HERE
        c = choose_CC(p, numRobots, myrobot.c)
        counter = 0

        while ((np.sum(np.abs(np.subtract(c, myrobot.c)))>1 and np.sum(np.abs(np.subtract(c, myrobot.c)))!=0.0) and counter<50): # detect a minimal change of cluster centroids
        # while (((np.sum(np.abs(np.subtract(c, myrobot.c)))>1) or np.sum(np.abs(np.subtract(c, myrobot.c)))==0.0) and counter<10): # detect a minimal change of cluster centroids
            # print np.sum(np.abs(np.subtract(c, myrobot.c))), np.sum(np.abs(np.subtract(c, myrobot.c)))>1, np.sum(np.abs(np.subtract(c, myrobot.c)))!=0.0, np.all(c==myrobot.c), counter<50

            # c = choose_CC(p, numRobots, myrobot.c)
            # print 'in while loop', counter, (np.sum(np.abs(np.subtract(c, myrobot.c))))
            for particle in p:
                p_array = np.array([particle.xs, particle.ys, particle.Q, particle.Dy, particle.Dz, particle.v, particle.H, particle.a])
                # cluster assignment (find which centroid is closest)
                minD = []
                for mu in c: # iterate over the rows 
                    # print 'mu', mu
                    # print 'particle:', p_array
                    # print 'subtract:', np.power(np.subtract(mu, p_array),2.0)
                    # print 'sqrt:',np.sqrt(np.power(np.subtract(mu, p_array),2.0))
                    # print 'sum:',np.sum(np.sqrt(np.power(np.subtract(mu, p_array),2.0)))
                    minD.append(np.sum(np.sqrt(np.power(np.subtract(mu, p_array),2.0)))) # 2-norm distance                    
                    # print minD
                # print 'minD', minD
                # print np.argmin(minD)
                particle.cluster = np.argmin(minD)
                # print 'cluster', particle.cluster
                # raw_input()
            # raw_input()
            # reevaluate the centroid of the cluster 
            myrobot.c = c
            c,clusterP_stdev = get_clustered_centroids(p,numRobots)
            # raw_input()

            counter += 1
        myrobot.old_clusteredData = myrobot.clusteredData
        myrobot.clusteredData = c
        
        # assignment problem using the munkres algorthim
        # https://pypi.python.org/pypi/munkres/#downloads

        distCost = [[0 for i in range(myrobot.numRobot)] for j in range(myrobot.numRobot)]
        for i in range(myrobot.numRobot):
            for j in range(myrobot.numRobot):
                distCost[i][j] =sqrt((myrobot.clusteredData[j,0]-myrobot.x[i])**2.0+(myrobot.clusteredData[j,1]-myrobot.y[i])**2.0)
            
        matrix = distCost
        m = Munkres()
        indexes = m.compute(matrix)
        # print_matrix(matrix, msg='Lowest cost through this matrix:')
        total = 0
        # print indexes
        for row, column in indexes:
            value = matrix[row][column]
            total += value
            # print '(%d, %d) -> %d' % (row, column, value)
            # print 'total cost: %d' % total
            
        # print 'max at: ', 'x=', xs+cos(a)*H**2.0*v/(4*Dz), 'y=', ys+sin(a)*H**2.0*v/(4*Dz)
        # go towards the highest concentration
        # find maximum concentrations
        
        # # clusterData: xs, ys, Q, v, Dy, Dz, H, a
        for i,j in indexes:
            x_cl = myrobot.clusteredData[j,0]
            y_cl = myrobot.clusteredData[j,1]
            v_cl = myrobot.clusteredData[j,3]
            Dz_cl = myrobot.clusteredData[j,5]
            H_cl = myrobot.clusteredData[j,6]
            a_cl = myrobot.clusteredData[j,7]
                         # ys+sin(a)*H**2.0*v/(4*Dz)
            y_hc = y_cl + sin(a_cl)*H_cl**2.0*v_cl/(4*Dz_cl)
                       # xs+cos(a)*H**2.0*v/(4*Dz)
            x_hc = x_cl + cos(a_cl)*H_cl**2.0*v_cl/(4*Dz_cl)
            
            direction.append(atan2(y_hc-myrobot.y[i], x_hc-myrobot.x[i])) # go in direction of average source location     


            # print 'maxC:', x_hc, y_hc
            # print sin(a_cl)*H_cl**2.0*v_cl/(4*Dz_cl), H_cl, v_cl, Dz_cl
            # print cos(a_cl)*H_cl**2.0*v_cl/(4*Dz_cl)

        # raw_input()
        # go towards the source location
        for i,j in indexes:
            direction.append(atan2(myrobot.clusteredData[j,1]-myrobot.y[i], myrobot.clusteredData[j,0]-myrobot.x[i])) # go in direction of average source location     



        # for i in range(numRobots):
            # direction.append(atan2(myrobot.clusteredData[i,1]-myrobot.y[i], myrobot.clusteredData[i,0]-myrobot.x[i])) # go in direction of average source location     
        # raw_input()

    else:
        direction.append(pi/2.0)

    # print 'directon vector', direction



    # if my stdev is below 20 then I have a good estimate so i should just go towards it
    if stdev<20 and stg != 'ESL' and stg != 'clustering':
        print 'GOING TOWARDS THE SOURCE'
        xs,ys,Q,v,Dy,Dz,H,a = get_states(p)
        direction = []
        for i in range(numRobots):
            direction.append(atan2(sum(ys)/len(ys)-myrobot.y[i],sum(xs)/len(xs)-myrobot.x[i])) # go in direction of average source location     
            myrobot.msg[i]='GOING TOWARDS THE SOURCE'


    # angle = [(b*180.0/pi)%360 for b in direction]
    # print 'angle set', angle
    # print 'est. wind:', a*180.0/pi
    
    # print ''
    # raw_input('getting direction')
    # print ''

    
    return direction

def get_clustered_centroids(particles,numRobots):
    cont = np.zeros((numRobots,8))
    # print 'container:', cont
    # raw_input()
    N = len(particles)
    numb_C_particles =np.zeros(numRobots)

    y_unit = np.zeros(numRobots)
    x_unit = np.zeros(numRobots)
    for p in particles:
        # print p.cluster, p
        cont[p.cluster,0]+=p.xs 
        cont[p.cluster,1]+=p.ys
        cont[p.cluster,2]+=p.Q
        cont[p.cluster,3]+=p.v
        cont[p.cluster,4]+=p.Dy
        cont[p.cluster,5]+=p.Dz
        cont[p.cluster,6]+=p.H
        # cont[p.cluster,7]+=p.a
        y_unit[p.cluster]+=np.sin(p.a)
        x_unit[p.cluster]+=np.cos(p.a)

        numb_C_particles[p.cluster]+=1
    for i in range(numRobots):
        cont[i,7] = np.arctan2(y_unit[i],x_unit[i])%(2.0*pi) 
    # for i in range(len(a)):
        # y_unit += np.sin(a[i])
        # x_unit += np.cos(a[i])
    # ave_a = np.arctan2(y_unit,x_unit)%(2.0*pi)

    # print ''
    # print 'number of particles in clusters', numb_C_particles
    # print ''
    
    # print 'xs, ys, Q, v, Dy, Dz, H, a'
    # print 'before division'
    # print cont 
    # print ''
    # cont = np.divide(cont, N)
    
    # print numb_C_particles

    clusterP_stdev = 0
    ave_clusterP = np.sum(numb_C_particles)/numRobots
    # print ave_clusterP
    
    for n in numb_C_particles:
        # print n
        # print ave_clusterP
        clusterP_stdev = clusterP_stdev + (n-ave_clusterP)**2.0
        # print clusterP_stdev
    clusterP_stdev = sqrt(clusterP_stdev/(numRobots-1))
    # print 'clusteredP_stdev ', clusterP_stdev
    # raw_input()
    # use this to know when to reinitialize the centroid!!!!???
    

    for i in range(numRobots): # getting the average for each cluster
        if numb_C_particles[i] == 0:
            cont[i,:]=cont[i,:]
        else:
            cont[i,0:7]=np.divide(cont[i,0:7],numb_C_particles[i])

    # print 'after division'
    # print cont[0, 0:6]
    # print cont
    

    # cluster_stdev = np.zeros(numRobots) # stdev for each cluster
    # for p in particles:
        # cluster_stdev[p.cluster] = cluster_stdev[p.cluster] + (p[i].xs-ave_xs)**2 + (p[i].ys-ave_ys)**2 + (p[i].Q-ave_Q)**2 + (p[i].v-ave_v)**2 + (p[i].Dy-ave_Dy)**2 + (p[i].Dz-ave_Dz)**2 + (p[i].H-ave_H)**2 + (p[i].a-ave_a)**2
    # stdev = sqrt(stdev/(N-1))
    
    # print cont
    # raw_input()
    return cont, clusterP_stdev




extension = '.eps'
myrobot = particlePlume(realPlume=1, numRobot=3)
# predicted_noise = 0.002
# real_noise = 0.0005
predicted_noise = 0.002
real_noise = 0.001
# Note my noise assumptions require knowledge of the real plume, because 0.001 may be low noise for a given real plume or really high noise for a different real plume
# not sure how to handle this, because If i randomly init a real plume, if my real_noise and predicted_noise don't correspond with the real plume then it will be difficult to estimate a model.

# ESL - estimated source location, RBW - random biase walk, SMA - silkworm moth algorithm,  clustering - k-means clusting to test multiple hypothesises
default = 'clustering'
noise_ratio = predicted_noise/real_noise
# must have noise_ratio>0 or bad estimate 
print 'noise_ratio:', noise_ratio
myrobot.set_noise(0.5,0.5,real_noise)

debug = False

fig2=plt.figure()
plt.ion()

# from mpl_toolkits.mplot3d import Axes3D
N = 2000
Nt = N*.6
T = 200
p = []

# initalizae all particles states randomly and set noise
for i in range(N):
    r = particlePlume(realPlume=0)
    r.set_noise(0.5,0.5,predicted_noise) # forward noise, turning noise, sensor noise
    p.append(r)

w = range(N) # initalized weights
plot_PF(p,myrobot,t=0,w=w)
mp = None
dflag = 0
cflag = 0
Z = myrobot.sense(myrobot)
stdev = 10000000
store_data = np.zeros(8)
store_robot = np.zeros(2)
error = np.zeros(6)


param_stdev = np.zeros(6)
for t in range(1,T+1):
    print t
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

    # store data and plot error
    row=np.array((ave_xs, ave_ys, ave_Q, ave_v, ave_Dy, ave_Dz, ave_H, ave_a))
    store_data = np.vstack([store_data, row])
    for i in range(myrobot.numRobot):
        store_robot= np.vstack([store_robot, (myrobot.x[i], myrobot.y[i])])
        
    stdev = 0
    param_row = (0,0,0,0,0,0)
    for i in range(N):
        # need to have a standard vector for parameters so I can change them with ease.
        
        # print (p[i].xs-ave_xs)**2 + (p[i].ys-ave_ys)**2#, (p[i].Q-ave_Q)**2, (p[i].v-ave_v)**2,(p[i].Dy-ave_Dy)**2 + (p[i].Dz-ave_Dz)**2, (p[i].H-ave_H)**2, (p[i].a-ave_a)**2
        # param_row = ((p[i].xs-ave_xs)**2 + (p[i].ys-ave_ys)**2, (p[i].Q-ave_Q)**2, (p[i].v-ave_v)**2, (p[i].Dy-ave_Dy)**2 + (p[i].Dz-ave_Dz)**2,  (p[i].H-ave_H)**2, (p[i].a-ave_a)**2)
        stdev = stdev + (p[i].xs-ave_xs)**2 + (p[i].ys-ave_ys)**2 + (p[i].Q-ave_Q)**2 + (p[i].v-ave_v)**2 + (p[i].Dy-ave_Dy)**2 + (p[i].Dz-ave_Dz)**2 + (p[i].H-ave_H)**2 + (p[i].a-ave_a)**2
        param_row=np.add(((p[i].Q-ave_Q)**2, (p[i].v-ave_v)**2, (p[i].Dy-ave_Dy)**2 + (p[i].Dz-ave_Dz)**2,  (p[i].H-ave_H)**2, (p[i].a-ave_a)**2, (p[i].xs-ave_xs)**2 + (p[i].ys-ave_ys)**2), param_row)
        

        if w[i] == mw:
            mp = p[i]
            # print i, p[i], w[i]
        if sw != 0:
            w[i] = w[i]/sw
        
        if w[i] != 0:
            # print 'w:',w[i]
            N_eff += (w[i]**2)
    
    param_row = np.sqrt(param_row/(N-1))
    stdev = sqrt(stdev/(N-1))
    param_stdev = np.vstack([param_stdev, param_row])
    # print 'param shape', param_stdev.shape
    # print param_stdev
    error = np.vstack([error, eval(myrobot,p)])
    # print 'error shape', error.shape
    # print error
    # raw_input()

    # plot_error(myrobot,p, error, store_robot, t, param_stdev)


    # print 'stdev:', stdev
    
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
    plot_PF(p,myrobot,t, mp,w)



    # np.savetxt('data.txt', store_data, delimiter=',')
