#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import numpy as np

_ACTIONS = [('N', [0,-1]),('E', [-1,0]),('S',[0,1]),('W',[1,0]),('NE',[-1,-1]),('NW',[1,-1]),('SE',[-1,1]),('SW',[1,1]), ('STAY',[0,0])]
_X = 0
_Y = 1

class refinementNode():

    def __init__(self):
        pub = rospy.Publisher('action', String, queue_size=1)
        rospy.Subscriber("offlineAction", String, self.offlineActioncallback)
        rospy.Subscriber("gradientAction", String, self.gradientActioncallback)

        rospy.init_node('refinementNode', anonymous=True)
        loopRate = rospy.get_param('Lrate', 10)
        rate = rospy.Rate(loopRate/2)
        self.offlineAction = "STAY"
        self.gradientAction = "STAY"
        while not rospy.is_shutdown():
            # action = np.random.choice(_ACTIONS)
           # action='NE'

            pub.publish(self.offlineAction)
            # pub.publish(self.gradientAction)
            # pub.publish(Action)
            rate.sleep()

    def gradientActioncallback(self, data):
        self.gradientAction = data

    def offlineActioncallback(self, data):
        self.offlineAction = data


refinementNode()
