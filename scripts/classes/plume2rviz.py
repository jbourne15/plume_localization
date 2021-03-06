#!/usr/bin/env python

import numpy as np
import math

# myclasses
# import plumeModel
from plumeModel import gaussPlume 

#ros specific 
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, MapMetaData, GridCells
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray



class rosMapper(object):
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
        if self.type=='gaussian':
            self.plumeMap = gaussPlume(xs=0,ys=50,Q=1,Dy=1,Dz=.5,v=1,h=5,originX=0,originY=0,resolution=300\
                                       ,width=100,height=100)
        else:# defaulyt is gaussPlume
            self.plumeMap = gaussPlume(xs=0,ys=3,Q=1,Dy=1,Dz=.5,v=1,h=10,originX=0,originY=0,resolution=10\
                                       ,width=10,height=10)
           
        #set the concentration values for the grid, self.plumeMap.grid is ready for occupancyGrid msg type (0-1)
        # Latched publishers are used for slow changing topics like
        # maps.  Data will sit on the topic until someone reads it.
        self.plumeMap_pub = rospy.Publisher('map', OccupancyGrid, latch=True, queue_size=1)
        self.plumeMap_data_pub = rospy.Publisher('Cmap_metadata', MapMetaData, latch=True, queue_size=1)
        self.concentration_pub = rospy.Publisher('concentration', Float64MultiArray, queue_size=1)
        self.sendConcentration = Float64MultiArray() # init
        # rospy.Subscribyer("state", PointStamped, self.Statecallback)

    def publish_map(self):
        """ Publish the map. """
        Cgrid = self.plumeMap.get_grid() 
        grid_msg = self.to_message(Cgrid)
        # print 'grid_msg', grid_msg
        # print ''
        self.plumeMap_data_pub.publish(grid_msg.info)
        self.plumeMap_pub.publish(grid_msg)

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
    # this will run publish a concentration map for a given plume model (gaussian,etc.) 
    m = rosMapper()
    loopRate = 10
    rate = rospy.Rate(loopRate) # must be must faster to avoid using old information
    m.publish_map()
    print 'published'
    while not rospy.is_shutdown():
        # m.concentration_pub.publish(m.sendConcentration)
        # m.publish_map() # only need to publish once because this is static values
        rate.sleep()
