#!/usr/bin/env python

import rospy

import matplotlib.pyplot as plt
import numpy as np
import math
from std_msgs.msg import String

from nav_msgs.msg import OccupancyGrid, MapMetaData, GridCells
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray

_ACTIONS = [('N', [0,-1]),('E', [-1,0]),('S',[0,1]),('W',[1,0]),('NE',[-1,-1]),('NW',[1,-1]),('SE',[-1,1]),('SW',[1,1]), ('STAY',[0,0])]
# _ACTIONS = [('N', [0,1]),('E', [1,0]),('S',[0,-1]),('W',[-1,0]),('NE',[1,1]),('NW',[-1,1]),('SE',[1,-1]),('SW',[-1,-1]), ('STAY',[0,0])]

_X = 0
_Y = 1

# Q = 25.0/1000
#
# xs = 0
# ys = 25
#
# xmin = 10
# xmax = 50
# ymin = 1
# ymax = 50
#
# [x,y] = np.meshgrid(np.linspace(xmin,xmax,100), np.linspace(ymin,ymax,100))
# Dy = 1
# Dz = .5
# v = 1
# H = 5
# xx = x-xs
# yy = y-ys
#
# c=Q/(2*np.pi*xx*math.sqrt(Dy*Dz))*np.exp(-H**2*v/(4*Dz*xx))*np.exp(-v*yy**2/(4*Dy*xx))
#
# plt.contourf (x,y,c)
# plt.colorbar()
#
# plt.show()

class Map(object):
    """
    The Map class stores an occupancy grid as a two dimensional
    numpy array.

    Public instance variables:

        width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters.
        origin_x   --  Position of the grid cell (0,0) in
        origin_y   --    in the map coordinate system.
        grid       --  numpy array with height rows and width columns.


    Note that x increases with increasing column number and y increases
    with increasing row number.
    """
    def __init__(self, origin_x=0, origin_y=0, resolution=1,
                 width=100, height=100):
        """ Construct an empty occupancy grid.

        Arguments: origin_x,
                   origin_y  -- The position of grid cell (0,0) in the
                                map coordinate frame.
                   resolution-- width and height of the grid cells
                                in meters.
                   width,
                   height    -- The grid will have height rows and width
                                columns cells.  width is the size of
                                the x-dimension and height is the size
                                of the y-dimension.

         The default arguments put (0,0) in the center of the grid.

        """
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width))
    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """

        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "Cmap"

        # .info is a nav_msgs/MapMetaData message.
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height

        # Rotated maps are not supported... quaternion represents no
        # rotation.
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        flat_grid = self.grid.reshape((self.grid.size,))
        grid_msg.data = list(np.round(flat_grid))
        return grid_msg
    def set_cells(self):
        """ Set the value of a cell in the grid.
            values is the corresponding concentration mapped to a value between 0-100
        """
        # emission rate
        Q = rospy.get_param("Q", 1)

        #stack location
        xs = rospy.get_param("xs", 0)
        ys = rospy.get_param("ys", self.width/2)

        #wind velocity and stack height
        v = rospy.get_param("v", 1)
        H = rospy.get_param("H", 5)

        #diffusivities
        Dy = rospy.get_param("Dy", 1)
        Dz = rospy.get_param("Dz", .5)

        #grid
        xmin = 0.0000000000000001
        xmax = 10
        ymin = 0.0000000000000001
        ymax = 10

        # xmin = 0.0000000000000001
        # xmax = self.width
        # ymin = 0.0000000000000001
        # ymax = self.height

        # the resolution of the plume is encoded into the self.height, and self.width term this creates a finer meshgrid below
        # the actual size of the plume grid is the 50x50 which is specified by the xmax and ymax terms above



        [x,y] = np.meshgrid(np.linspace(xmin,xmax,self.height), np.linspace(ymin,ymax,self.width))
        # [x,y] = np.meshgrid(np.linspace(xmin,xmax,self.resolution), np.linspace(ymin,ymax,self.resolution))

        self.c = np.zeros((len(x),len(y)))

        for i in range(0,len(x)):
            for j in range(0,len(y)):
                xx = x[i,j] - xs
                yy = y[i,j] - ys
                self.c[j,i] = Q/(2*np.pi*xx*math.sqrt(Dy*Dz))*np.exp(-H**2*v/(4*Dz*xx))*np.exp(-v*yy**2/(4*Dy*xx))

        print np.size(self.c,0), np.size(self.c,0)


        Cmax = np.ndarray.max(self.c)
        Cmin = np.ndarray.min(self.c)
        # map to the 0-100 range for the Occupancygrid msg
        for i in range(0,len(x)):
            for j in range(0,len(y)):
                self.grid[i,j] = self.remapping(self.c[i,j], Cmin, Cmax, 0, 100)
                # self.grid[j,i] = 100 # black
                # self.grid[j,i] = 0 # white
        #         print self.grid[i,j], self.c[i,j], i,j
        # print ''
        # print self.c
        # print ''
        # print self.c[0,0], self.c[5,5], self.c[0,5], self.c[5,0], Cmax, Cmin
        # print ''
        # print self.grid
        # print ''
        # print self.grid[0,0], self.grid[5,5], self.grid[0,5], self.grid[5,0]



    def remapping(self,x, inmin, inmax, outmin, outmax):
        return (x-inmin)*(outmax-outmin)/(inmax-inmin)+outmin
class Mapper(object):

    def __init__(self,w,h,r=1):
        """ Start the mapper. """

        rospy.init_node('Concentration_mapper')
        # self._map = Map(origin_x=0, origin_y=0, width=w,height=h, resolution=r)
        self._map = Map(width=w,height=h)
        # self._map = Map(width=w,height=h, resolution=r)

        # Latched publishers are used for slow changing topics like
        # maps.  Data will sit on the topic until someone reads it.
        self._map_pub = rospy.Publisher('map', OccupancyGrid, latch=True, queue_size=1)
        self._map_data_pub = rospy.Publisher('map_metadata', MapMetaData, latch=True, queue_size=1)

        self.concentration_pub = rospy.Publisher('concentration', Float64MultiArray, queue_size=1)
        rospy.Subscriber("state", PointStamped, self.Statecallback)
        self.sendConcentration = Float64MultiArray() # init

    def publish_map(self):
        """ Publish the map. """
        self._map.set_cells()
        grid_msg = self._map.to_message()
        self._map_data_pub.publish(grid_msg.info)
        self._map_pub.publish(grid_msg)
        # print grid_msg
    def Statecallback(self, state):
        # print state.point
        statei = PointStamped()

        if state.point.x == self._map.width: # error handling 0-99 not 1-100
            state.point.x = self._map.width-1
        if state.point.y == self._map.width:
            state.point.y = self._map.height-1

        self.sendConcentration = Float64MultiArray() # reset
        # self.sendConcentration.data.append(self._map.c[state.point.x, state.point.y])
        for a,i in _ACTIONS:
            statei.point.x = state.point.x + i[_X]
            statei.point.y = state.point.y + i[_Y]

            if statei.point.x>=self._map.height:
                statei.point.x = self._map.height-1
            if statei.point.y>=self._map.width:
                statei.point.y = self._map.width-1

            print 'state:',state.point.x, state.point.y, 'Action:', a, i
            print 'state_i',statei.point.x, statei.point.y
            print 'C(si):',self._map.c[statei.point.y, statei.point.x]
            print ''
            self.sendConcentration.data.append(self._map.c[statei.point.y, statei.point.x]) # had to switch for visualization [y,x]
            # statei = state
            # print statei.point, a
            # print self._map.c[statei.point.x, statei.point.y]

        print self.sendConcentration.data

# 5 by 5 (self.height = self.width = 6)
# [[ 0.          0.          0.          0.          0.          0.        ]
#  [ 0.00046141  0.00508623  0.01688688  0.01688688  0.00508623  0.00046141]
#  [ 0.00553158  0.01836549  0.0334641   0.0334641   0.01836549  0.00553158]
#  [ 0.01063378  0.02366592  0.0353054   0.0353054   0.02366592  0.01063378]
#  [ 0.01354297  0.02467691  0.03331034  0.03331034  0.02467691  0.01354297]
#  [ 0.01488617  0.02405716  0.03058264  0.03058264  0.02405716  0.01488617]]


        # print "concentration = ", self._map.c[state.point.x, state.point.y]

w = rospy.get_param("width", 100)
h = rospy.get_param("height", 100)
# r = rospy.get_param("resolution", 100)
# m = Mapper(w,h,r)

m = Mapper(w,h)
m._map.set_cells()
loopRate = rospy.get_param('Lrate', 10)
rate = rospy.Rate(loopRate*2) # must be must faster to avoid using old information
m.publish_map()
while not rospy.is_shutdown():
    m.concentration_pub.publish(m.sendConcentration)
    rate.sleep()

    # <node pkg="rviz" type="rviz" name="rviz" args="--display-config $(find plume_localization)/rviz/plume_localization.rviz"/>