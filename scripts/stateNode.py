#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

_ACTIONS = [('N', [0,-1]),('E', [-1,0]),('S',[0,1]),('W',[1,0]),('NE',[-1,-1]),('NW',[1,-1]),('SE',[-1,1]),('SW',[1,1]), ('STAY',[0,0])]
# _ACTIONS = [('N', [0,1]),('E', [1,0]),('S',[0,-1]),('W',[-1,0]),('NE',[1,1]),('NW',[-1,1]),('SE',[1,-1]),('SW',[-1,-1]), ('STAY',[0,0])]
_X = 0
_Y = 1

class stateNode():

    def __init__(self):
        pub = rospy.Publisher('state', PointStamped, queue_size=10)
        markerPub = rospy.Publisher('robotMarker', Marker, queue_size=1)
        rospy.Subscriber("action", String, self.move_callback)

        rospy.init_node('stateNode', anonymous=True)
        loopRate = rospy.get_param('Lrate', 10)
        rate = rospy.Rate(loopRate*2) #must be faster loop for states to be correct
        self.state = PointStamped()

        # initial starting location I might want to move to the param list
        # self.r = rospy.get_param("resolution",100)
        # self.h = rospy.get_param("height", 100)*self.r
        # self.w = rospy.get_param("width", 100)*self.r

        # initial starting location I might want to move to the param list
        self.h = rospy.get_param("height", 100)
        self.w = rospy.get_param("width", 100)
        # self.r = rospy.get_param("resolution", 100)

        self.state.point.x = self.h-.5 # shift position to visualize onto the grid
        self.state.point.y = self.w-.5 # shift position

        # self.state.point.x = self.h*self.r-.5 # shift position to visualize onto the grid
        # self.state.point.y = self.w*self.r-.5 # shift position
        self.state.point.z = 0

        while not rospy.is_shutdown():

            sendState = PointStamped()
            sendState.point.x = self.state.point.x +.5 - 1
            sendState.point.y = self.state.point.y +.5 - 1
            sendState.point.z = 0
            print "sending state:", sendState.point
            pub.publish(sendState)

            self.robotMarker = Marker()
            self.robotMarker.header.frame_id = "Cmap"
            self.robotMarker.header.stamp    = rospy.Time.now()
            self.robotMarker.ns = "robot"
            self.robotMarker.id = 0
            self.robotMarker.type = Marker.SPHERE
            self.robotMarker.action = Marker.ADD
            self.robotMarker.pose.position = self.state.point
            self.robotMarker.pose.position.z = .5 # shift sphere up
            self.robotMarker.pose.orientation.x = 0
            self.robotMarker.pose.orientation.y = 0
            self.robotMarker.pose.orientation.z = 0
            self.robotMarker.pose.orientation.w = 1.0
            self.robotMarker.scale.x = 1.0
            self.robotMarker.scale.y = 1.0
            self.robotMarker.scale.z = 1.0
            # self.robotMarker.scale.x = 1.0*self.ry
            # self.robotMarker.scale.y = 1.0*self.r
            # self.robotMarker.scale.z = 1.0*self.r

            self.robotMarker.color.r = 1.0
            self.robotMarker.color.g = 0.0
            self.robotMarker.color.b = 0.0
            self.robotMarker.color.a = 1.0

            markerPub.publish(self.robotMarker)
            # print "sending marker", self.robotMarker.pose.position
            rate.sleep()

    def move_callback(self, action):
        for i in _ACTIONS:
            if String(i[0]) == action:

                self.state.point.x = self.state.point.x + i[1][_X]
                self.state.point.y = self.state.point.y + i[1][_Y]

                if self.state.point.x>self.h:
                    self.state.point.x = self.h-.5 # shift sphere marker
                if self.state.point.y>self.w:
                    self.state.point.y = self.w-.5 # shift sphere marker
                self.state.point.z = 0

stateNode()
