#!/usr/bin/env python

from PID import PID
import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from std_msgs.msg import Bool
from math import *
import time

myAccelerate = PID(0.000005,0,0.0000025)
myDecelerate = PID(0.0022,0.0035,0.0000035) #the higher, the faster the slow down
class Simulator:
    def __init__(self):
        self.next_goal=0
        self.goal=[0,0]
        self.lane0, self.lane1 = self.lanes(0.5, 2, 0, 10)
        self.lanelength = len(self.lane0)
        self.error = 0
        self.topspeed = 5
        self.vconst = 0
        self.stop = False
    
    def stopSignReceived(self, data):
        self.stop = data.data
    

    def listen(self, cmdmsg, cmdpub):
        rospy.Subscriber('/stop-sign-detected',Bool, self.stopSignReceived)
        rospy.Subscriber('odometry/filtered',nav_msgs.msg.Odometry,self.huskyOdomCallback, 
                 (cmdpub,cmdmsg))

    def pid(self):
        prv_error = self.error
        current_time = time.time()
        time.sleep(0.001)
        error = self.topspeed - self.vconst
        etime = time.time() - current_time
        self.vconst = self.vconst + myAccelerate.pid_controller(error,prv_error,etime)

    def pid_decelerate(self):
        prv_error = self.error
        current_time = time.time()
        time.sleep(0.001)
        error = self.topspeed - self.vconst
        etime = time.time() - current_time
        self.vconst = self.vconst + myDecelerate.pid_controller(error,prv_error,etime)

    def goal_switch(self, select):
        switch={0: [20000,0], 
                1: [10000,0],
                2: [1000, 0],
                3: [10, 0],
                4: [9, 0],
                5: [0, 0]}
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
        print("stopping = %s ", self.stop)
        # Callback arguments 
        pub,msg = cargs
        stopDistance = 1.8
	    
        self.pid()
        # Tunable parameters
        wgain = 15.0 # Gain for the angular velocity [rad/s / rad]
        vconst = self.vconst # Linear velocity when far away [m/s]
        distThresh = 0.25 # Distance treshold [m]
        #print("vconst: {0}".format(vconst))

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
        #print(self.goal, pose[1], middle, pose[0], self.lanelength)
        # Proportional Controller
        v = 0 # default linear velocity
        w = 0 # default angluar velocity

        destination = True
        distance = sqrt((pose[0]-self.goal[0])**2+(pose[1]-self.goal[1])**2)

        if(self.stop):
            print("Seeing Stop Sign. Stopping now...")
            self.topspeed = 0
            self.pid_decelerate()

        # if(distance < 1.8):
        #     self.topspeed = 0
        #     self.pid_decelerate()
       
        if (distance > distThresh):
            destination = False

        
        distance = sqrt((pose[0]-self.goal[0])**2+(pose[1]-self.goal[1])**2)
        #print(distance)
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
rospy.spin()
