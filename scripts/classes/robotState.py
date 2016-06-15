#!/usr/bin/env python

import numpy as np
import math

#ROS specific 
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
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
        self.robotMarker.pose.position.z =-.5*reso+5 #.5*reso # shift sphere up
        self.robotMarker.pose.orientation.x = 0
        self.robotMarker.pose.orientation.y = 0
        self.robotMarker.pose.orientation.z = 0
        self.robotMarker.pose.orientation.w = 1.0
        self.robotMarker.scale.x = 1*reso
        self.robotMarker.scale.y = 1*reso
        self.robotMarker.scale.z = 1*reso
        self.robotMarker.color.r = 0.0
        self.robotMarker.color.g = 0.0
        self.robotMarker.color.b = 1.0
        self.robotMarker.color.a = 1.0

        # pub = rospy.Publisher('state', PointStamped, queue_size=1)
        self.markerPub = rospy.Publisher('robotMarker', Marker, queue_size=1)
        

        
        self.path = Marker()

        self.width = width
        self.height = height 
        self.reso = reso

        self.path.header.frame_id = "Cmap"
        self.path.header.stamp    = rospy.Time.now()
        self.path.ns = "robot"
        self.path.id = 0
        self.path.type = Marker.POINTS
        self.path.action = Marker.ADD

        # self.path.pose.position.x =  self.state[0]
        # self.path.pose.position.y = self.state[1]
        # self.path.pose.position.z = 0
        # self.path.pose.orientation.x = 0
        # self.path.pose.orientation.y = 0
        # self.path.pose.orientation.z = 0
        # self.path.pose.orientation.w = 1.0

        self.path.scale.x = 1*reso
        self.path.scale.y = 1*reso
        self.path.scale.z = 1*reso
        self.path.color.r = 1.0
        self.path.color.g = 0.0
        self.path.color.b = 0.0
        self.path.color.a = .25
        # self.path.lifetime = rospy.Duration(1)
        
        
        self.pointPub = rospy.Publisher('path', Marker, queue_size=1)

    def set_robotState(self,state):
        # print 'setting robot state:', state.data
        self.robotMarker.pose.position.x =(state.data[1]+.5)*self.reso
        self.robotMarker.pose.position.y =(state.data[0]+.5)*self.reso
        
        point = Point()
        
        point.y = (state.data[0]+.5)*self.reso
        point.x = (state.data[1]+.5)*self.reso

        # self.path.color.g = self.path.color.g+0.0001

        self.path.points.append(point)
        
    def publish_Marker(self):
        self.markerPub.publish(self.robotMarker)
        self.pointPub.publish(self.path)
