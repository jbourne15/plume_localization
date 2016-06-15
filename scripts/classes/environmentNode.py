#!/usr/bin/env python

import numpy as np
import math
from copy import deepcopy
from plumeModel import gaussPlume, linearPlume, randomPlume
from robotState import robotMarker

#ROS specific 
import rospy
from std_msgs.msg import String, Float64MultiArray, Int16MultiArray
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid, MapMetaData, GridCells
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PointStamped
# import rosbag




    # <node pkg="rviz" type="rviz" name="rviz" args="--display-config $(find plume_localization)/rviz/rviz_plume.rviz"/>

# _ACTIONS = [('N', [0,-1]),('E', [1,0]),('S',[0,1]),('W',[-1,0]),('NE',[1,-1]),('NW',[-1,-1]),\
            # ('SE',[1,1]),('SW',[-1,1]), ('STAY',[0,0])]

_ACTIONS = [('N', [-1,0]),('E', [0,1]),('S',[1,0]),('W',[0,-1]),('NE',[-1,1]),('NW',[-1,-1]),\
            ('SE',[1,1]),('SW',[1,-1]), ('STAY',[0,0])]
RECORDSTATE = 0

class environment(object):
    '''this classe takes a plumeModel class and sents it to rviz for visualization.  This class acts as the 
    wrapper around my plumeModel to separate that class from ROS so I only have to change this class for ROS
    updates.  This class also handles my robot state for visualization for ros.
    '''
    def __init__(self):
        rospy.init_node('environmentNode')
        # self.type = rospy.get_param("type", 'gaussian')
        self.type = 'linear'
        self.type = rospy.get_param('type', ['linear'])

        # gaussian constructor/init inputs method
        # xs, ys, Q, Dy, Dz, v, h, originX, originY, resolution, width, height
        
        self.width = rospy.get_param("width",100)
        self.height = rospy.get_param("height",100)
        self.resolution = rospy.get_param("resolution",100)

        if self.type=='gaussian':
            self.plumeMap = gaussPlume(xs=0,ys=self.width/2,Q=1,Dy=.5,Dz=.5,v=1,h=5,originX=0,originY=0,\
                                       resolution=self.resolution,width=self.width,height=self.height)
            # print self.plumeMap.get_conc()
            # print self.plumeMap.get_conc().shape
        elif self.type =='linear':
            # top
            self.plumeMap = linearPlume(originX=0, originY=0, resolution=self.resolution, width=self.width,\
                                        height=self.height, a=-1, b=0, c=-1, d=-200)

            # self.plumeMap = linearPlume(originX=0, originY=0, resolution=self.resolution, width=self.width,\
                                        # height=self.height, a=-1, b=0, c=1, d=-200)

            # top left
            # self.plumeMap = linearPlume(originX=0, originY=0, resolution=self.resolution, width=self.width,\
                                        # height=self.height, a=-1, b=-1, c=-1, d=-200)
            # top right 
            # self.plumeMap = linearPlume(originX=0, originY=0, resolution=self.resolution, width=self.width,\
                                        # height=self.height, a=-1, b=1, c=-1, d=-200)
            # bot right 
            # self.plumeMap = linearPlume(originX=0, originY=0, resolution=self.resolution, width=self.width,\
                                        # height=self.height, a=-1, b=-1, c=1, d=-200)
            # bot left 
            # self.plumeMap = linearPlume(originX=0, originY=0, resolution=self.resolution, width=self.width,\
                                        # height=self.height, a=-1, b=1, c=1, d=-200)
        elif self.type == 'random':
            self.plumeMap = randomPlume(originX=0,originY=0, resolution=self.resolution, width=self.width,\
                                        height=self.height, h=1, clx=100)
    
        # print self.plumeMap.get_conc().shape
        
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
        self.startState = [float(i)*self.resolution/(self.width*1.0) for i in self.startState]
        # print 'reso/wid',float(self.resolution/self.width), self.resolution, self.width

        if self.startState[0]>=self.resolution:
            self.startState[0]=self.resolution-1
        elif self.startState[0]<0:
            self.startState[0] = 0.0
        if self.startState[1]>=self.resolution:
            self.startState[1]=self.resolution-1
        elif self.startState[1]<0:
            self.startState[1] = 0.0

        if RECORDSTATE:    # if this is true the program will record the distribution of the number of times
                           # a state has been visited
            # self.bag = rosbag.Bag('mydata.bag','w')
            from collections import defaultdict
            self.stateHistory = np.array([])
            self.stateHistory = np.insert(self.stateHistory, 0, np.array(self.startState))
            self.stateDis = {tuple(self.startState): 1}
            self.stateDis = defaultdict(lambda: 0, self.stateDis)
            import time
            self.t1 = time.time()


        self.robotState.data.append(self.startState[0])
        self.robotState.data.append(self.startState[1]) # Y
        print 'state:',self.robotState.data
        self.robotMarker.set_robotState(self.robotState)

        loopRate = rospy.get_param('Lrate', 50)
        # print type(loopRate)
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
        
        # print 'stateHistory:', self.stateHistory
        if RECORDSTATE:
            self.stateDis[tuple(self.robotState.data)] = self.stateDis[tuple(self.robotState.data)] + 1
            self.stateHistory = np.insert(self.stateHistory,len(self.stateHistory),np.array(self.robotState.data))
            # print 'stateH:', self.stateHistory
            # print type(self.stateHistory), self.stateHistory.size, self.stateHistory.shape

            # print 'dis:',self.stateDis

            self.plotHisto()
        self.robotMarker.set_robotState(self.robotState)
        # print 'c[statei]:', C[self.robotState.data[0], self.robotState.data[1]]
        # print 'sendC:', self.sendConcentration
        # raw_input()

    def plotHisto(self):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        import numpy as np
        import time
        
        t2 = time.time()
        # print t2-self.t1
        c = 1
        x = []
        y = []
        print len(self.stateHistory)/2
        # if t2-self.t1 >= 30:p
        if len(self.stateHistory)/2>=5000:
            for i in self.stateHistory:
                if c%2 is 0:
                    #even
                    # print 'xs', i
                    x.append(i)
                else:
                    #odd    
                    # print 'ys', i
                    y.append(i)
                c = c+1
            
            # x.append(100)
            # y.append(100)
            # x.append(0)
            # y.append(100)
            # x.append(100)
            # y.append(0)
        
            x = np.array(x)
            y = np.array(y)
            
            
             # <!-- saves the data in ~/.ros folder -->
            np.savetxt('mydata.csv', (x,y), delimiter=',')            
            # t = h = o = np.arange(0.0,5.0,1.0)
            # np.savetxt('hello.csv', t, delimiter=',')   # x,y,z equal sized 1D arrayes
            # print 'saved'
            # print y

            # plt.hist2d(x,y,bins=self.resolution)
            # plt.colorbar()
            # plt.axis([0, self.width, 0, self.height])
            # # plt.gca().invert_xaxis()
            # plt.gca().invert_yaxis()
            # plt.show()
            '''
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            # x, y = np.random.rand(2, 100) * 4
            
            # print x
            print "plotting"
            hist, xedges, yedges = np.histogram2d(x, y, bins=50)

            elements = (len(xedges) - 1) * (len(yedges) - 1)

            xpos, ypos = np.meshgrid(xedges[:-1] + 0.25, yedges[:-1] + 0.25)

            
            xpos = xpos.flatten()
            ypos = ypos.flatten()
            zpos = np.zeros(elements)
            dx = 0.5 * np.ones_like(zpos)
            dy = dx.copy()
            dz = hist.flatten()

            ax.bar3d(xpos, ypos, zpos, dx, dy, dz, color='blue', zsort='average')

            plt.show()

            '''

            import numpy as np
            import matplotlib.mlab as mlab
            import matplotlib.pyplot as plt

            # plt.ion()
            # the histogram of the data

            # raw_input("ready to plot?")
            n, bins, patches = plt.hist(x, 100, normed=1, facecolor='green', alpha=0.75)
            plt.axis([0, 100, 0, .3])
            plt.grid(True)
            plt.show()
            
            n, bins, patches = plt.hist(y, 100, normed=1, facecolor='green', alpha=0.75)            
            plt.axis([0, 100, 0, .3])
            plt.grid(True)
            plt.show()



    def publish_map(self):
        """ Publish the map. """
        Cgrid = self.plumeMap.get_grid() 
        grid_msg = self.to_message(Cgrid)
        self.plumeMap_data_pub.publish(grid_msg.info)
        self.plumeMap_pub.publish(grid_msg)
        
    
    def publish_concentration(self):
        self.sendConcentration = Float64MultiArray()
        C =self.plumeMap.get_conc() # changed this to the normalize concentration for rviz 0 - 100

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
        # try:
        # print ''
        # print self.robotState
        # self.bag.write('state_bag',self.robotState)
        # print 'hi'
        # finally:
            # self.bag.close()
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
