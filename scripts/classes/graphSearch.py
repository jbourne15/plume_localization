#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Int16MultiArray
from geometry_msgs.msg import PointStamped
import heapq
import numpy as np
import math
import operator
import time

_ACTIONS = [('N', [-1,0]),('E', [0,1]),('S',[1,0]),('W',[0,-1]),('NE',[-1,1]),('NW',[-1,-1]),\
            ('SE',[1,1]),('SW',[1,-1]), ('STAY',[0,0])]

class Astar(object):

    def __init__(self,width=100,height=100, resolution=100, goal=(0,0), state=(0,0)):
        self.frontier = []

        # self.width = rospy.get_param("height", 100)
        # self.height = rospy.get_param("width", 100)
        
        self.width = width
        self.height = height
        self.resolution = resolution
        self.spacing = 1.0*self.width/self.resolution
        # print 'space:',self.spacing
        self.state = None
        # self.setGoal = None
        self.flag = 0 # this flag sets when I can and can't update my state in self.set_state
        # without this sometimes with I am in self.runPlanner the state subscriber would run its callback 
        # this eventually screws up the dictionary self.S2A

        self.set_state(state)
        self.set_goal(goal)


    def runPlanner(self):
        self.flag = 1
        
        n0 = (self.state, 'None', 0)

        if self.setGoal is not None:
            self.goal = self.setGoal

        if self.isGoalState(n0[0]):
            return

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
            if (ni[0], ni[1]) not in visited:
                visited.append((ni[0], ni[1], 0))
                if self.isGoalState(ni[0]):
                    action = self.getPath(ni,parentMap) # this methods sets policy for the states as well see method
                    # print 'here'
                    self.flag = 0
                    # return self.S2A
                    break
                else:
                    for i in self.getSuccessors(ni[0]):
                        if i not in visited:
                            cost = i[2] + cost_to_node + self.heuristic(i[0])
                            frontier.push(i,cost)
                            parentMap[i] = ni
            lpc = lpc +1


    def getCost(self, n,parentMap):
        cost = n[2]
        while (n[1] != 'None'):
            n = parentMap[n]
            cost = cost + n[2]
        return cost

    def getPath(self, n,parentMap):
        curr = n
        action = []
        states = []
        # print 'got here 2'
        while (curr != None):
            action.append(curr[1])
            states.append(curr[0])
            curr = parentMap[curr]
        action.reverse()
        action.remove('None')
        states.reverse()
        self.S2A = {states[0]:action[0]} # setting the policy
        for si,ai in zip(states, action):
            self.S2A[si] =ai
        self.S2A[self.goal] = 'STAY'
        
    def isGoalState(self, ni):
        if ni == self.goal:
            return True
        else:
            return False

    def getSuccessors(self,ni):
        state = ni
        childrenList = []
        # import pdb; pdb.set_trace()
        for a,i in _ACTIONS:
            statei = tuple(map(operator.add, state, tuple(i)))            
            if statei == state:
                t = 'dont add this state'
            else:
                if statei[0]>=self.resolution or statei[0]<0 or statei[1]>=self.resolution or statei[1]<0:
                    t = 'out of bounds'
                else:
                    childrenList.append((statei,a,1))
        return childrenList

    def heuristic(self,state):
        heuristic = math.fabs(self.goal[0]-state[0])+math.fabs(self.goal[1]-state[1]) # manhattan distance
        return heuristic

    def set_state(self, state):
        self.stateFlag = 1
        # if self.flag == 0:
            # self.state = tuple([np.round(self.spacing*x) for x in state])
        self.state = state
        # print 'setting state:', self.state # -----------
        self.stateFlag = 0


    def set_goal(self, goal):
        self.setGoal = list(goal)
        self.setGoal = [np.round(x/self.spacing) for x in goal] # make sure goal is in same units!!!!

        # if self.resolution == self.width:
        if self.setGoal[0]>=self.resolution: # constraining goal to be within the state space defined
            self.setGoal[0]=self.resolution-1
        elif self.setGoal[0]<0:
            self.setGoal[0] = 0.0
        if self.setGoal[1]>=self.resolution:
            self.setGoal[1]=self.resolution-1
        elif self.setGoal[1]<0:
            self.setGoal[1] = 0.0

        self.setGoal = tuple(self.setGoal)        
        print 'goal changed to:', self.setGoal

    def get_action(self):
        if self.isGoalState(self.state):
            return 'STAY'
        elif self.flag == 1 or self.stateFlag == 1:
            # print 'still running planner'
            return 'STAY'
        else:
            # sometimes my dictionary doesn't have self.state in it so i get a keyerror.
            # this happens when I have a faster looprate and my planner can't keep up.  
            # not sure how to fix it other than putting it in a try except block.
            
            try:
                action = self.S2A[self.state]
            except (KeyError):
                action = 'STAY'
            return action


# ------------------------------------------------------------------------------- #
        
class DstarLite(Astar):
      def __init__(self,width=100, height=100, resolution=100, goal=(0,0), state=(100,100)):

          super(DstarLite, self).__init__(width,height, resolution, goal, state)
          # self.state = state
          # self.goal = goal
          print 'HI DSTARLITE'
          # print self.goal
          # self.sLast = self.state
          self.init()
          self.ComputeShortestPath()
      def init(self):
          self.km=0
          # if self.setGoal!=None:
              # self.goal = self.setGoal
              
         
          if self.setGoal is not None:
              self.goal = self.setGoal

          self.rhs={self.goal:0}
          from collections import defaultdict # setting up default value for dictionary
          self.goalDist = {None: None}
          self.goalDist = defaultdict(lambda: 10000000, self.goalDist)
          self.rhs = defaultdict(lambda: 10000000, self.rhs)
          self.U = PriorityQueue()
          self.U.push(self.goal,self.calKey(self.goal))

      def calKey(self,s):
          return [min(self.goalDist[s],self.rhs[s])+self.heuristic(self.state,s)+self.km,min(self.goalDist[s],self.rhs[s])]

      def heuristic(self,s,sp):
          return math.fabs(s[0]-sp[0])+math.fabs(s[1]-sp[1]) # manhattan distance
    
      def ComputeShortestPath(self):
          while(self.U.topKey()<self.calKey(self.state) or self.rhs[self.state] != self.goalDist[self.state]):
              # print 'computing shortest path'
              k_old = self.U.topKey()
              u = self.U.pop()
              # print 'U is:', self.U.heap
              # print 'expanded into:', u
              if k_old<self.calKey(u):
                  self.U.push(u,self.calKey(u))
              elif self.goalDist[u]>self.rhs[u]:
                  self.goalDist[u] = self.rhs[u]
                  for s in self.Pred(u): self.updateVertex(s)
                  # print 'here 1'
              else:
                  self.goalDist[u] = 10000000
                  pred = self.Pred(u)
                  pred.append(u)
                  for s in pred: self.updateVertex(s)
                  # print 'here 2'


      def Pred(self,u):
          state = u
          parentList = []
          for a,i in _ACTIONS:
              statei = tuple(map(operator.add, state, tuple(i)))            
              if statei == state:
                  t = 'dont add this state'
              else:
                  if statei[0]>=self.width or statei[0]<0 or statei[1]>=self.width or statei[1]<0:
                      t = 'out of bounds'
                  else:
                      parentList.append((statei))
          # print 'si',u
          # print 'pred or succ',parentList
          return parentList

      def Succ(self,u):
          return self.Pred(u)

      def updateVertex(self,u):
          if u != self.goal:
              hold = []
              for s in self.Succ(u): 
                  hold.append(1+self.goalDist[s]) # cost of 1 
              self.rhs[u] = min(hold)
        
          if self.U.exists(u):
              self.U.remove(u)
          if self.goalDist[u] != self.rhs[u]:
              state = u
              self.U.push(u,self.calKey(u))
              # print 'U is:',self.U.heap
              # print 'pushed new state', u

      def runPlanner(self):
          self.flag=1
          if self.setGoal is not None:
              self.goal = self.setGoal

          if self.isGoalState(self.state):
              # self.S2A[self.state] = 'STAY'
              return

          # while(self.state!=None and self.goal!=None): # wait for init state and goal to be initalized
              # time.sleep(10)
              # print 'stuck here'

              
          # sLast = self.state
          # self.init()
          # self.ComputeShortestPath()

          # while self.state != self.goal:
          if self.goalDist[self.state]==10000000: # no path exists
              return 'Error no path exists'
          mini = 1000000
          for s in self.Succ(self.state):
              f = 1+self.goalDist[s]
              if f< mini:
                  mini = f
                  miniState = s
          self.getPath(miniState)
          self.flag = 0
                                      
          # import pdb; pdb.set_trace()
          # scan for edge costs changed
          '''
          if any edge costs changed
          km = km + self.heuristic(sLast, self.state)
          sLast = self.state
          for all directed edges (u,v) with changed edge costs:
          update the edge cost c(u,v) # probably need to make this a directory so I can update it
          i only used the value 1 for this 
          
          updateVertex(u)
          
          ComputeShortestPath()              
          
          '''
              
      def getPath(self,ministate):
          
          print ministate
          print self.state
          if self.state == ministate:
              return self.S2A
          else:
              self.S2A = 0
              for a,i in _ACTIONS:
                  statei = tuple(map(operator.add, self.state, tuple(i)))            
                  if statei == ministate:
                      self.S2A = {self.state:a}
          
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
        self.s = set()
    def push(self, item, priority):
        # FIXME: restored old behaviour to check against old results better
        # FIXED: restored to stable behaviour
        entry = (priority, self.count, item)
        self.s.add(item)
        # entry = (priority, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        #  (_, item) = heapq.heappop(self.heap)
        self.s.remove(item)
        return item

    def topKey(self):
        (p, c, item) = heapq.heappop(self.heap)
        self.push(item,p)
        return p

    def isEmpty(self):
        return len(self.heap) == 0

    def exists(self, item):
        return item in (x[2] for x in self.heap)

    def remove(self, item):
        i = 0
        for x in self.heap:
            if item == x[2]:
                itr = i 
            i=i+1
        del self.heap[itr]



if __name__=="__main__":
    pathPlanner = DstarLite(4,4,4)
    # pathPlanner = Astar(100,100,100)
    pathPlanner.set_state((3,3))
    pathPlanner.set_goal((0,0))
    pathPlanner.runPlanner()
