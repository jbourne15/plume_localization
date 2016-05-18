#!/usr/bin/env python

import numpy as np
import math

#ROS specific 
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

# _ACTIONS = [('N', [0,-1]),('E', [-1,0]),('S',[0,1]),('W',[1,0]),('NE',[-1,-1]),('NW',[1,-1]),\
            # ('SE',[-1,1]),('SW',[1,1]), ('STAY',[0,0])]

_ACTIONS = [('N', [-1,0]),('E', [0,1]),('S',[1,0]),('W',[0,-1]),('NE',[-1,1]),('NW',[-1,-1]),\
            ('SE',[1,1]),('SW',[1,-1]), ('STAY',[0,0])]



class robotMarker(object):
    def __init__(self,width, height, reso):
        # self.width = width-.5 # from before
        # self.height = height-.5 # from before
        self.width = width
        self.height = height 
        self.reso = reso
                      # x,            y,      z
        # self.state  = [width*reso-.5*reso,height*reso-.5*reso,.5*reso]
        
        # self.state = [reso-.5*reso, reso-.5*reso]

        # self.state = [0+.5*reso,0+.5*reso]


        # self.state =[self.state[0]+2*reso, self.state[1]+2*reso]

        # print 'robotmarker state:', self.state

        self.robotMarker = Marker()
        self.robotMarker.header.frame_id = "Cmap"
        self.robotMarker.header.stamp    = rospy.Time.now()
        self.robotMarker.ns = "robot"
        self.robotMarker.id = 0
        self.robotMarker.type = Marker.SPHERE
        self.robotMarker.action = Marker.ADD
        # self.robotMarker.pose.position.x =  self.state[0]
        # self.robotMarker.pose.position.y = self.state[1]
        self.robotMarker.pose.position.z = 0#.5*reso # shift sphere up
        self.robotMarker.pose.orientation.x = 0
        self.robotMarker.pose.orientation.y = 0
        self.robotMarker.pose.orientation.z = 0
        self.robotMarker.pose.orientation.w = 1.0
        self.robotMarker.scale.x = 1*reso
        self.robotMarker.scale.y = 1*reso
        self.robotMarker.scale.z = 1*reso
        self.robotMarker.color.r = 1.0
        self.robotMarker.color.g = 0.0
        self.robotMarker.color.b = 0.0
        self.robotMarker.color.a = 1.0

        # pub = rospy.Publisher('state', PointStamped, queue_size=1)
        self.markerPub = rospy.Publisher('robotMarker', Marker, queue_size=1)
        
    def set_robotState(self,state):
        # print 'setting robot state:', state.data
        self.robotMarker.pose.position.x =(state.data[1]+.5)*self.reso
        self.robotMarker.pose.position.y =(state.data[0]+.5)*self.reso
            # self.robotMarker.pose.position.x = state.data[0]
            # self.robotMarker.pose.position.y = state.data[1]
        
    def publish_Marker(self):
        self.markerPub.publish(self.robotMarker)

        # def moveCallback(self,action):

            # for i in _ACTIONS:
                # if String(i[0]) == action:
                    # self.state.point.x = self.state.point.x + i[1][_X]
                    # self.state.point.y = self.state.point.y + i[1][_Y]

                    # if self.state.point.x>self.h:
                        # self.state.point.x = self.h-.5 # shift sphere marker
                    # if self.state.point.y>self.w:
                        # self.state.point.y = self.w-.5 # shift sphere marker
                        # self.state.point.z = 0

        # def publishConcentrations(self, state):
            # print state.point
            # statei = PointStamped()
            
            # if state.point.x == self._map.width: # error handling 0-99 not 1-100
                # state.point.x = self._map.width-1
            # if state.point.y == self._map.width:
                # state.point.y = self._map.height-1

            # self.sendConcentration = Float64MultiArray() # reset
            # self.sendConcentration.data.append(self._map.c[state.point.x, state.point.y])
            # for a,i in _ACTIONS:
                # statei.point.x = state.point.x + i[_X]
                # statei.point.y = state.point.y + i[_Y]

            # if statei.point.x>=self._map.height:
                # statei.point.x = self._map.height-1
            # if statei.point.y>=self._map.width:
                # statei.point.y = self._map.width-1

            # print 'state:',state.point.x, state.point.y, 'Action:', a, i
            # print 'state_i',statei.point.x, statei.point.y
            # print 'C(si):',self._map.c[statei.point.y, statei.point.x]
            # print ''
            # self.sendConcentration.data.append(self._map.c[statei.point.y, statei.point.x]) # had to switch for visualization [y,x]
            # statei = state
            # print statei.point, a
            # print self._map.c[statei.point.x, statei.point.y]
            # print self.sendConcentration.data



# if __name__=="__main__":
    # this will run publish a concentration map for a given plume model (gaussian,etc.) 

    # loopRate = 10
    # rate = rospy.Rate(loopRate) # must be must faster to avoid using old information

    # while not rospy.is_shutdown():


        # rate.sleep()
