#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Int16MultiArray
from geometry_msgs.msg import PointStamped
from graphSearch import Astar
import numpy as np
import math

_ACTIONS = [('N', [-1,0]),('E', [0,1]),('S',[1,0]),('W',[0,-1]),('NE',[-1,1]),('NW',[-1,-1]),\
            ('SE',[1,1]),('SW',[1,-1]), ('STAY',[0,0])]


class actionNode():

    def __init__(self):
        pub = rospy.Publisher('action', String, queue_size=1)
        rospy.Subscriber("concentration", Float64MultiArray, self.concentrationCallback)
        rospy.init_node('actionNode')
        
        loopRate = rospy.get_param('Lrate', 50)/2
        rate = rospy.Rate(loopRate)

        self.width = rospy.get_param("width",100)
        self.height = rospy.get_param("height",100)
        self.resolution = rospy.get_param("resolution",100)

        rospy.Subscriber("/clicked_point", PointStamped, self.goal_callback)
        rospy.Subscriber("state", Int16MultiArray, self.state_callback)
        
        w = rospy.get_param('w',[0,1]) # gradientW, plannerW
        w = w.split()
        w = [float(i) for i in w]
        print w

        self.set_actionWeights(w=np.array(w)) # these are the weights I change gradientW, plannerW
        self.sendAction = 'STAY'
        self.gradientAction = 'STAY'

        self.probCount = np.zeros(2) # keeps track of the probability of which action was taken
        
        self.goal = rospy.get_param('g', [0, 0])
        self.goal = self.goal.split()
        self.goal = [float(i) for i in self.goal]

        if self.goal[0]>=self.resolution: # constraining goal to be within the state space defined
            self.goal[0]=self.resolution-1
        elif self.goal[0]<0:
            self.goal[0] = 0.0
        if self.goal[1]>=self.resolution:
            self.goal[1]=self.resolution-1
        elif self.goal[1]<0:
            self.goal[1] = 0.0
        

        self.planner = Astar(self.width, self.height, self.resolution) # this sets up what graph search technique i am using
        self.planner.set_goal(tuple(self.goal)) # init goal location

        while not rospy.is_shutdown():
            if self.planner.state is not None: # wait for state to be intialized
                self.planner.runAstar()
                self.set_sendAction_PB() # sampling of actions
                # self.set_sendAction_WC() # linear combination of actions
                pub.publish(self.sendAction)
            rate.sleep()

    def concentrationCallback(self, concentration): 
        # this callback function finds the action in the steepest gradient
        max_A = []
        if len(concentration.data)>0:
            i = 0
            c_so = concentration.data[len(concentration.data)-1] # last concentration value is at current state from (STAY)
            max_dC = -1000000
            test = []
            
            C =  list(concentration.data)
            maxG = max([x-c_so for x in C])

            for c_si in concentration.data:
                dC = -c_so + c_si
                if dC == maxG: # changes from >= to >
                    # max_dC = dC
                    max_A.append(_ACTIONS[i][0])
                    # max_A = _ACTIONS[i]
                i = i + 1
            max_A = np.array(max_A)
            self.gradientAction = np.random.choice(max_A)

            # for c_si in concentration.data:
            #     dC = -c_so + c_si
            #     if dC > max_dC: # changes from >= to >
            #         max_dC = dC
            #         # max_A.append(_ACTIONS[i][0])
            #         max_A = _ACTIONS[i]
            #     i = i + 1
            
            # print concentration.data
            # self.gradientAction = max_A[0]
            # print self.gradientAction
            
            self.set_actionWeights(c=c_so)
            

    def goal_callback(self, goal):
        setGoal = np.rint(np.array((goal.point.y-1, goal.point.x-1))/self.resolution)
        
        if setGoal[0]<0:
            setGoal[0]=0
        elif setGoal[0]>=self.width:
            setGoal[0] = self.width-1

        if setGoal[1]<0:
            setGoal[1]=0
        elif setGoal[1]>=self.height:
            setGoal[1] = self.height-1

        self.planner.set_goal(tuple(setGoal))

    def state_callback(self, state):
        setState = state.data
        # print 'state',setState
        self.planner.set_state(setState)
    
    def set_sendAction_WC(self): # this takes the linear combination of the actions and the weights
        self.sendAction = 'STAY'
        gA = [0,0]
        pA = [0,0]
        for a, i in _ACTIONS: #extract unit vectors for each action (self.gradientAction and planner action) from _ACTIONS
            if a == self.gradientAction:
                gA = i
            if a == self.planner.get_action():
                pA = i

        A =np.array([gA, pA]) 
        wA = np.zeros(np.shape(A))
        # print wA
        for i in range(0,len(A)): # create weights matrix for the actions
            wA[i] = A[i]*self.weights[i]
        print self.gradientAction, self.planner.get_action()
        print wA
        tA = 0
        for i in range(0,len(A)): # sum over all weighted actions
            tA = tA + wA[i]
        # print np.linalg.norm(tA), np.linalg.norm(tA) !=0
        print tA

        if math.isnan(np.linalg.norm(tA)) is not True and np.linalg.norm(tA) != 0:
            tA = np.round(tA/np.linalg.norm(tA))

        for a, i in _ACTIONS:
            if i == list(tA):
                self.sendAction = a
        print tA, self.sendAction

        stg = 'none'
        if self.sendAction == self.gradientAction:
            stg = 'gradientAction'
        elif self.sendAction == self.planner.get_action():
            stg = 'plannerAction'
        if self.sendAction == self.planner.get_action() and self.sendAction == self.gradientAction:
            stg = 'both'
        if self.sendAction != self.planner.get_action() and self.sendAction != self.gradientAction:
            stg = 'neither'
        
        # print 'gA:', self.gradientAction, 'pA:', self.planner.get_action(), 'sendA:', self.sendAction, stg, tA
    def set_sendAction_PB(self): 
        # this chooses the likely action depending on the weights or the probability that an action is selected.
        # If there is action in between the gradientAction and the plannerAction then take the middle action if weighted the same
        # if there is multple actions in between the gradientAction and the plannerAction then reduce the action
        # to the closes inbetween action and then either take the middle action if weigthed the same or choose the action base off of the prob.
        
        self.sendAction = 'STAY'
        gA = [0,0]
        pA = [0,0]
        for a, i in _ACTIONS: #extract unit vectors for each action (self.gradientAction and planner action) from _ACTIONS
            if a == self.gradientAction:
                gA = i
            if a == self.planner.get_action():
                pA = i

        r = np.random.random_sample() # uniform distribution sampling
        # print self.probCount

        if r<self.weights[0]:
            self.sendAction = self.gradientAction
            # print "gradientAction"
            self.probCount[0] = self.probCount[0]+1
        else:
            self.sendAction = self.planner.get_action()
            # print "plannerAction"
            self.probCount[1] = self.probCount[1]+1

        stg = 'none'
        if self.sendAction == self.gradientAction:
            stg = 'gradientAction'
        elif self.sendAction == self.planner.get_action():
            stg = 'plannerAction'
        if self.sendAction == self.planner.get_action() and self.sendAction == self.gradientAction:
            stg = 'both'
        if self.sendAction != self.planner.get_action() and self.sendAction != self.gradientAction:
            stg = 'neither'
        
        # print 'gA:', self.gradientAction, 'pA:', self.planner.get_action(), 'sendA:', self.sendAction, stg

        
        # print self.probCount/sum(self.probCount)
        
        # print self.gradientAction, self.planner.get_action(), self.sendAction
        # zaxis = np.array([0,0,1])
        # gA = np.array(gA)
        # gA = np.append(gA,0)
        # pA = np.array(pA)
        # pA = np.append(pA,0)
        # print gA, np.cross(zaxis, gA), pA,np.cross(zaxis, pA)
        
    def set_actionWeights(self,w=None, c=None):

        if w is not None: # init
            self.weights = w
            
        if c is not None: # dependent on concetration values
            # if c>50:
            #     self.weights = np.array((.8, .2))
            #     print 'gradient', c
            # elif c<=50:
            #     self.weights = np.array((.2, .8))
            #     print 'planner', c
            # print c
            wg = 1/100.0*c
            wp = 1-wg
            print wg, wp
            self.weights = np.array((wg,wp))
            

if __name__=="__main__":
    actionNode()
