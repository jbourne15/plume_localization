#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped
_ACTIONS = ['N','E','S','W','NE','NW','SE','SW','STAY']


class gradientNode():

    def __init__(self):
        pub = rospy.Publisher('gradientAction', String, queue_size=10)
        rospy.Subscriber("state", PointStamped, self.stateCallback)
        rospy.Subscriber("concentration", Float64MultiArray, self.concentrationCallback)
        rospy.init_node('gradientNode', anonymous=True)
        loopRate = rospy.get_param('Lrate', 10)
        rate = rospy.Rate(loopRate)

        self.sendAction = 'STAY'
        self.state = PointStamped

        while not rospy.is_shutdown():
            pub.publish(self.sendAction)
            rate.sleep()

    def concentrationCallback(self, concentration):

        if len(concentration.data)>0:
            i = 0
            # print concentration.data, len(concentration.data)
            c_so = concentration.data[len(concentration.data)-1] # last concentration value is at current state from (STAY)
            print "current C:", c_so, max(concentration.data)
            max_dC = -1000000
            test = []
            for c_si in concentration.data:
                dC = -c_so + c_si
                print c_so, c_si,_ACTIONS[i], dC
                if dC >= max_dC:
                    max_dC = dC
                    max_A  = _ACTIONS[i]
                i = i + 1
            print ''
            print max_dC, max_A
            print concentration.data
# 5 by 5 (self.height = self.width = 6)
# [[ 0.          0.          0.          0.          0.          0.        ]
#  [ 0.00046141  0.00508623  0.01688688  0.01688688  0.00508623  0.00046141]
#  [ 0.00553158  0.01836549  0.0334641   0.0334641   0.01836549  0.00553158]
#  [ 0.01063378  0.02366592  0.0353054   0.0353054   0.02366592  0.01063378]
#  [ 0.01354297  0.02467691  0.03331034  0.03331034  0.02467691  0.01354297]
#  [ 0.01488617  0.02405716  0.03058264  0.03058264  0.02405716  0.01488617]]

            self.sendAction = max_A
    def stateCallback(self,state):
        self.state = state
        print ''
        print 'state:', state.point.x, state.point.y

gradientNode()