#!/usr/bin/env python

import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep

class Simulator:
    def __init__(self):
        self.next_goal=0
        self.goal=[0,0]
        self.lane0, self.lane1 = self.lanes(0.5, 2, 0, 10)
        self.lanelength = len(self.lane0)

    '''
        Note: 1. pass in a tuple, because list is unhashable.
              2. current_position needs to be a tuple consisting of two points.
              3. current_position needs to exist in the dictionary of tuples

        _checkpoints will be modified with points from gazebo
    '''

    def waypoint(self, current_position):
        _checkpoints = {
            (0, 0): [(0, 1), (4, 8), (5, 9)],
            (0, 1): [(3, 7), (2, 6)],
            (4, 8): [(3, 7)],
            (5, 9): [(2, 6)],
            (3, 7): [(0, 0), (2, 6), (5, 9)],
            (2, 6): [(4, 8)],
        }

        point = _checkpoints[current_position]
        index = random.randint(1, len(point))  # it will randomly choose whether to go straight, turn left, or right
        goal_position = point[index - 1]

        self.goal[0] = goal_position[0]
        self.goal[1] = goal_position[1]

    def listen(self, cmdmsg, cmdpub):
        rospy.Subscriber('odometry/filtered',nav_msgs.msg.Odometry,self.huskyOdomCallback, 
                 (cmdpub,cmdmsg))

        rospy.spin()

    def goal_switch(self, select):
        switch={0: [12,14], 
                1: [10,-2],
                2: [10,-5],
                3: [10, -6],
                4: [9, -7],
                5: [0, -8]}
        self.goal = switch.get(select)
    
    def lanes(self,width1, width2, len1, len2):
        x1 = []
        x2 = []
        for i in range(len1,len2):
            x1.append([i,width1])
            x2.append([i,width2])
        
        middle = (x1[0][1]+ x2[0][1])/2

        return x1, x2


    def huskyOdomCallback(self, message,cargs):
        # Implementation of proportional position control 
        # For comparison to Simulink implementation

        # Callback arguments 
        pub,msg = cargs


        # Tunable parameters
        wgain = 15.0 # Gain for the angular velocity [rad/s / rad]
        vconst = 5.0 # Linear velocity when far away [m/s]
        distThresh = 0.25 # Distance treshold [m]

        # Generate a simplified pose
        pos = message.pose.pose
        quat = pos.orientation
        # From quaternion to Euler
        angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,
                                                           quat.z,quat.w))
        theta = angles[2]
        pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta
        middle = 0
        
        self.goal_switch(self.next_goal)
        print(self.goal, pose[1], middle, pose[0], self.lanelength)
        # Proportional Controller
        v = 0 # default linear velocity
        w = 0 # default angluar velocity
	
	destination = True
	distance = sqrt((pose[0]-self.goal[0])**2+(pose[1]-self.goal[1])**2)
        print(distance)
        if (distance > distThresh):
		destination = False

        if(pose[0] <= 8.5 and destination == False):
	    middle = 0
            self.goal[1] = middle
            self.goal[0] = pose[0] + 0.5

	if(pose[0] > 8.5 and destination == False):
       	    middle = 12
	    self.goal[0] = middle
            self.goal[1] = pose[1] + 0.5
            print("In lane")
           
        
        distance = sqrt((pose[0]-self.goal[0])**2+(pose[1]-self.goal[1])**2)
        print(distance)
        if (distance > distThresh):
            v = vconst
            desireYaw = atan2(self.goal[1]-pose[1],self.goal[0]-pose[0])
            u = desireYaw-theta
            bound = atan2(sin(u),cos(u))
            w = min(0.5 , max(-0.5, wgain*bound))

        #if (distance < distThresh):
        #    self.next_goal = self.next_goal + 1
        #    print ("NEXT, {}", self.next_goal)
        # Publish
        msg.linear.x = v
        msg.angular.z = w
        pub.publish(msg)
        
        # Reporting
        print('huskyOdomCallback: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f, goal[0]=%4.2f, goal[1]=%4.2f'%(pose[0],pose[1],distance,v,w,self.goal[0], self.goal[1]))

########################################
# Main Script
# Initialize our node
rospy.init_node('nre_simhuskycontrol',anonymous=True)
    
# Set waypoint for Husky to drive to
next_goal=1
goal = 0 #goal_switch(next_goal)


# Setup publisher
cmdmsg = geometry_msgs.msg.Twist()
cmdpub = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)

# Setup subscription - which implemets our controller.
# We pass the publisher, the message to publish and the goal as 
# additional parameters to the callback function.
#rospy.Subscriber('odometry/filtered',nav_msgs.msg.Odometry,huskyOdomCallback, 
#                 (cmdpub,cmdmsg,goal,next_goal))


sim = Simulator()
sim.listen(cmdmsg, cmdpub)
# spin() simply keeps python from exiting until this node is stopped
#rospy.spin()
