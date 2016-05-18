#!/usr/bin/env python

import numpy as np
import math
from copy import deepcopy
from plumeModel import gaussPlume 
from robotState import robotMarker

#ROS specific 
import rospy
from std_msgs.msg import String, Float64MultiArray, Int16MultiArray
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid, MapMetaData, GridCells
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PointStamped


    # <node pkg="rviz" type="rviz" name="rviz" args="--display-config $(find plume_localization)/rviz/rviz_plume.rviz"/>

# _ACTIONS = [('N', [0,-1]),('E', [1,0]),('S',[0,1]),('W',[-1,0]),('NE',[1,-1]),('NW',[-1,-1]),\
            # ('SE',[1,1]),('SW',[-1,1]), ('STAY',[0,0])]

_ACTIONS = [('N', [-1,0]),('E', [0,1]),('S',[1,0]),('W',[0,-1]),('NE',[-1,1]),('NW',[-1,-1]),\
            ('SE',[1,1]),('SW',[1,-1]), ('STAY',[0,0])]


class environment(object):
    '''this classe takes a plumeModel class and sents it to rviz for visualization.  This class acts as the 
    wrapper around my plumeModel to separate that class from ROS so I only have to change this class for ROS
    updates.  This class also handles my robot state for visualization for ros. (I still need to make class for
    robot state)
    '''
    def __init__(self):
        rospy.init_node('environmentNode')
        # self.type = rospy.get_param("type", 'gaussian')
        self.type = 'gaussian'
        # gaussian constructor/init inputs method
        # xs, ys, Q, Dy, Dz, v, h, originX, originY, resolution, width, height
        
        self.width = rospy.get_param("width",100)
        self.height = rospy.get_param("height",100)
        self.resolution = rospy.get_param("resolution",100)

        if self.type=='gaussian':
            self.plumeMap = gaussPlume(xs=0,ys=self.width/2,Q=1,Dy=.5,Dz=.5,v=1,h=5,originX=0,originY=0,\
                                       resolution=self.resolution,width=self.width,height=self.height)
            print self.plumeMap.get_conc()
            # print self.plumeMap.get_conc().shape

        else:# default is gaussPlume
            self.plumeMap = gaussPlume(xs=0,ys=self.width/2,Q=1,Dy=1,Dz=.5,v=1,h=10,originX=0,originY=0,\
                                       resolution=self.resolution,width=self.width,height=self.height)
                                    
        self.plumeMap_pub = rospy.Publisher('map', OccupancyGrid, latch=True, queue_size=1)
        self.plumeMap_data_pub = rospy.Publisher('Cmap_metadata', MapMetaData, latch=True, queue_size=1)
        self.concentration_pub = rospy.Publisher('concentration', Float64MultiArray, queue_size=1)
        self.robotState_pub = rospy.Publisher('state', Int16MultiArray, queue_size=1)
        self.robotMarker = robotMarker(self.width,self.height, self.resolution)

        rospy.Subscriber("action", String, self.action_callback)

        self.sendConcentration = Float64MultiArray() # init
        self.robotState = Int16MultiArray()

        # Robot Start state

        self.startState = rospy.get_param('s0', [0, 0])
        self.startState = self.startState.split()
        self.startState = [float(i) for i in self.startState]

        # print self.startState, type(self.startState)

        if self.startState[0]>=self.resolution:
            self.startState[0]=self.resolution-1
        elif self.startState[0]<0:
            self.startState[0] = 0.0
        if self.startState[1]>=self.resolution:
            self.startState[1]=self.resolution-1
        elif self.startState[1]<0:
            self.startState[1] = 0.0

        # print self.startState, type(self.startState)

        self.robotState.data.append(self.startState[0])
        self.robotState.data.append(self.startState[1]) # Y
        # self.robotState.data.append(0) # ID
        print self.robotState.data
        self.robotMarker.set_robotState(self.robotState)

        loopRate = rospy.get_param('Lrate', 20)
        rate = rospy.Rate(loopRate) # must be must faster to avoid using old information
        self.publish_map()
        # print 'published concentration map'
        while not rospy.is_shutdown():
            self.publish_state()
            self.publish_concentration()
            self.robotMarker.publish_Marker()
            rate.sleep()

    def action_callback(self,action):

        statei = deepcopy(self.robotState)
        for a, i in _ACTIONS:
            if a == action.data:
                statei.data[0] = self.robotState.data[0]+i[0]
                statei.data[1] = self.robotState.data[1]+i[1]

        if statei.data[0]>=self.resolution:
            statei = deepcopy(self.robotState)
        elif statei.data[0]<0:
            statei = deepcopy(self.robotState)
        if statei.data[1]>=self.resolution:
            statei = deepcopy(self.robotState)
        elif statei.data[1]<0:
            statei = deepcopy(self.robotState)

        C =self.plumeMap.get_conc()
        # print 'state0', self.robotState.data
        # print 'c[state0]:', C[self.robotState.data[0], self.robotState.data[1]]
        self.robotState = statei
        # print action.data,'statei', self.robotState.data
        self.robotMarker.set_robotState(self.robotState)
        # print 'c[statei]:', C[self.robotState.data[0], self.robotState.data[1]]
        # print 'sendC:', self.sendConcentration
        # raw_input()

    def publish_map(self):
        """ Publish the map. """
        Cgrid = self.plumeMap.get_grid() 
        grid_msg = self.to_message(Cgrid)
        self.plumeMap_data_pub.publish(grid_msg.info)
        self.plumeMap_pub.publish(grid_msg)
   
    
    def publish_concentration(self):
        self.sendConcentration = Float64MultiArray()
        C =self.plumeMap.get_conc()
        for a, i in _ACTIONS:
            statei = deepcopy(self.robotState.data)
            statei[0] = statei[0]+i[0]
            statei[1] = statei[1]+i[1]
            
            if statei[0]>=self.resolution:
                statei = deepcopy(self.robotState.data)
            elif statei[0]<0:
                statei = deepcopy(self.robotState.data)
            if statei[1]>=self.resolution:
                statei = deepcopy(self.robotState.data)
            elif statei[1]<0:
                statei = deepcopy(self.robotState.data)
                
            # print ''
            # print 'si',  statei[0], statei[1]
            # print 'C[si]', C[statei[0],statei[1]]
            # print ''
            
            self.sendConcentration.data.append(C[statei[0],statei[1]])
            # print self.sendConcentration
            # raw_input()

        self.concentration_pub.publish(self.sendConcentration)
        # print 'send C',self.sendConcentration
        # raw_input()

    def publish_state(self):
        # print self.robotState
        self.robotState_pub.publish(self.robotState)
        
    def to_message(self, grid):# this is my wrapper method for all of my child classes for various plume models such as gaussian, etc.
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "Cmap"

        # .info is a nav_msgs/MapMetaData message.
        grid_msg.info.resolution = self.plumeMap.resolution
        grid_msg.info.width =self.plumeMap.resolution# self.plumeMap.width
        grid_msg.info.height =self.plumeMap.resolution# self.plumeMap.height

        # Rotated maps are not supported... quaternion represents no
        # rotation.
        grid_msg.info.origin = Pose(Point(self.plumeMap.originX, self.plumeMap.originY, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        flat_grid = grid.reshape((grid.size,))
        grid_msg.data = list(np.round(flat_grid))
        return grid_msg

if __name__=="__main__":
    environment()




