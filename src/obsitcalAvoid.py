#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from geometry_msgs.msg import Twist




class avoidO:

    def __init__(self):

        self.start = False

        self.twist = Twist()

        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0; self.twist.linear.z = 0

        self.U = 0
        self.V = 0 
        self.k = 0

        pub = rospy.Publisher('cmd_vel_O', Twist, queue_size = 1)
        rospy.init_node('avoidObstical', anonymous=True)
        rospy.Subscriber("cmd_vel", Twist, self.callback)
       
        self.rate = rospy.Rate(1) # ROS Rate at 5Hz


        while not rospy.is_shutdown():
            
            if self.start:

                rospy.sleep(0.05)

 
    def callback(self,msg):
        
        self.start = True
        dev = np.shape(msg.ranges)[0]/18
        res = tuple(grouper(dev, msg.ranges)) 
        minRanges = []

        for i in res:
            temp = np.array(i)
            minr =  min(temp[np.nonzero(temp)])
            minRanges.append(minr)
        
        print np.shape(minRanges)


    def grouper(self, n, iterable): 
        args = [iter(iterable)] * n 
        return zip(*args) 



if __name__=="__main__":

    avoidO()
    rospy.spin()