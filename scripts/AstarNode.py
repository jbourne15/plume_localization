#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import heapq
import numpy as np
import math

_ACTIONS = [('N', [0,-1]),('E', [-1,0]),('S',[0,1]),('W',[1,0]),('NE',[-1,-1]),('NW',[1,-1]),('SE',[-1,1]),('SW',[1,1]), ('STAY',[0,0])]
_X = 0
_Y = 1

class AstarNode():

    '''
    this node's responsibility to find  a path to the Estimate goal location using a motion planning technique

    publishes:
    offlineAction

    subribes:
    state
    /clicked_point
    '''

    def __init__(self):
        pub = rospy.Publisher('offlineAction', String, queue_size=10)
        rospy.Subscriber("state", PointStamped, self.state_callback)
        rospy.Subscriber("/clicked_point", PointStamped, self.getGoal_callback)

        rospy.init_node('AstarNode', anonymous=True)
        loopRate = rospy.get_param('Lrate', 10)
        rate = rospy.Rate(loopRate*10)

        self.frontier = []

        self.goal = PointStamped()
        self.goal.point.x = 0
        self.goal.point.y = 0
        self.goal.point.z = 0


        self.sx = rospy.get_param("height", 100)
        self.sy = rospy.get_param("width", 100)

        # set init state, no action, cost
        n0 = (PointStamped(), 'None', 0)
        n0[0].point.x = self.sx-1
        n0[0].point.y = self.sy-1
        n0[0].point.z = 0

        frontier = PriorityQueue() # this is my search stack
        visited  = [] # this is my visited list
        frontier.push(n0,0)
                     #child,Parent
        parentMap   = {n0:None}
        action = []
        lpc = 0

        while len(frontier.heap)>0:
            ni = frontier.pop()
            cost_to_node = self.getCost(ni,parentMap)
            if ni[0] not in visited:
                visited.append(ni[0])
                if self.isGoalState(ni[0].point):
                    # print "found goal!"
                    action = self.getPath(ni,parentMap)
                    # print len(action)
                    # print action
                    # print len(visited), len(frontier.heap)
                    return action
                else:
                    # print self.getSuccessors(ni[0])
                    for i in self.getSuccessors(ni[0]):
                        if i not in visited:
                            cost = i[2] + cost_to_node + self.heuristic(i[0])
                            frontier.push(i,cost)
                            parentMap[i] = ni #(ni[0],ni[1])
                    # print "node:", ni,"cost_to_node:", cost_to_node, "  ", "frontier:", frontier.heap
            lpc = lpc +1
            # if lpc == 10:
            #     while True:
            #         test = 1

        # while not rospy.is_shutdown():
        #     action = "N"
        #
        #     pub.publish(action)
        #     rate.sleep()

    def getCost(self, n,parentMap):
        # print "n is", n
        cost = n[2]
        # print "cost is ", cost
        while (n[1] != 'None'):
            n = parentMap[n]
            # print "next node", n
            cost = cost + n[2]
            # print "cost is ", cost
        return cost

    def getPath(self, n,parentMap):
        curr = n
        action = []
        states = []
        while (curr != None):
            # print "curr", curr
            action.append(curr[1])
            states.append(curr[0])
            # curr = parentMap[curr[0]]
            curr = parentMap[curr]
        action.reverse()
        # action.pop() # get rid of first action
        action.remove('None')
        states.reverse()
        # print 'states:', states, len(states)
        # print 'actions:', action, len(action)
        return action

    def isGoalState(self, ni):
        if ni == self.goal.point:
            return True
        else:
            return False

    def getSuccessors(self,ni):
        state = ni
        statei = PointStamped()
        childrenList = []

        for a,i in _ACTIONS:
            statei.point.x = state.point.x + i[_X]
            statei.point.y = state.point.y + i[_Y]

            if statei.point.x>=self.sx: #(self.height)
                statei.point.x = self.sx-1
            if statei.point.y>=self.sy:
                statei.point.y = self.sy-1

            childrenList.append((statei,a,1)) # all cost are 1, I can change this later
            statei = PointStamped() # reset

        return childrenList


    def heuristic(self,state):
        h = math.fabs(self.goal.point.x-state.point.x)+math.fabs(self.goal.point.y-state.point.y) # manhattan distance

        return h

    def state_callback(self, state):
        self.state = state

    def getGoal_callback(self, goal):

        self.goal.point.x = np.floor(goal.point.x)
        self.goal.point.y = np.floor(goal.point.y)
        self.goal.point.z = 0 # 2D
        # print self.goal



class PriorityQueue:
    """
      Implements a priority queue data structure. Each inserted item
      has a priority associated with it and the client is usually interested
      in quick retrieval of the lowest-priority item in the queue. This
      data structure allows O(1) access to the lowest-priority item.

      Note that this PriorityQueue does not allow you to change the priority
      of an item.  However, you may insert the same item multiple times with
      different priorities.
    """
    def  __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        # FIXME: restored old behaviour to check against old results better
        # FIXED: restored to stable behaviour
        entry = (priority, self.count, item)
        # entry = (priority, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        #  (_, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

AstarNode()