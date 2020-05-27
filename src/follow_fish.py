#!/usr/bin/env python
# license removed for brevity

import rospy
from sensor_msgs.msg import Image
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Twist
import sys, select, termios, tty

import matplotlib.pyplot as plt

class fishBot():
    
    def __init__(self):

        self.started = False
        self.bridge = CvBridge()
        self.im = None

        self.u = 0
        self.v = 0 
        self.fgbg = cv.createBackgroundSubtractorKNN() 

        self.check = True
        self.deltaX = 200
        self.X1b = 0
        self.X1c = 0
        self.theta = 0
        self.cmax = None
        self.extBot2 = None

        self.Xk = 0# k  = i-1

        self.V = 50
        self.twist = Twist()
        self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0

        ## collabration
        ilowH, ilowS, ilowV = 0, 62  , 6
        ihighH, ihighS, ihighV = 179 , 255 ,208
        self.lower_hsv = np.array([ilowH, ilowS, ilowV])
        self.higher_hsv = np.array([ihighH, ihighS, ihighV])

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        
        rospy.init_node('Robofish', anonymous=True)
        rospy.Subscriber("usb_cam/image_raw", Image, self.callback_cam2)
        self.rate = rospy.Rate(10) # ROS Rate at 5Hz

        self.radArr = np.linspace(0, 1.8 * np.pi  , 8 )
        radios = 50
        self.circle = np.array([np.cos(self.radArr)*radios+320 , np.sin(self.radArr)*radios+240] )
        print self.circle
        plt.scatter(self.circle[0],self.circle[1]  )
        plt.show()


        while not rospy.is_shutdown():
        
            if self.started:

                #self.collaborate_im()

                #if self.find_dir(): break

                #print 'dir is : ', np.rad2deg(self.theta) , 'deg' 
                #print 'hi'
                self.rate.sleep()

        cv.destroyAllWindows()
        
    def callback_cam2(self, data):

        # Use cv_bridge() to convert the ROS image to OpenCV format

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

            if self.started == False:
                self.started = True

            self.im = cv_image

        except CvBridgeError, e:
            print e

    def find_dir(self):

        frame = self.im

        if frame is None: return True
            
        # Every color except white
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, self.lower_hsv, self.higher_hsv)
        result = cv.bitwise_and(frame, frame, mask=mask)
        fgmask = self.fgbg.apply(result)
        
        _, th1 = cv.threshold(result, 30, 255 , cv.THRESH_BINARY)
        gray = cv.cvtColor(th1, cv.COLOR_BGR2GRAY)

        # Find outer contours
        image, cnts, hierarchy = cv.findContours(gray, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        areaSUM, CCy , CCx , maxA , CxMax , CyMax ,maxC ,c = 0 , 0 ,0 ,0, 0 ,0,0,None

        for c in cnts:
            
            M = cv.moments(c)
            if  M['m00'] > 100 :

                cx = int(M['m10']/M['m00']) # x c.m
                cy = int(M['m01']/M['m00']) # y c.m
                areaSUM  = areaSUM + M['m00'] # sum over Ai
                CCx = CCx  + cx* M['m00'] # sum over x c.m
                CCy = CCy + cy*M['m00']# sum over y c.m

                if maxA < M['m00']: maxA  ,CxMax , CyMax , self.cmax = M['m00'] , cx ,cy ,c

        if areaSUM > 0 :CCx , CCy  = CCx / areaSUM ,CCy / areaSUM
        Xc = tuple(np.array([CCx, CCy], dtype= int).reshape(1, -1)[0])
        Xcmax = tuple(np.array([CxMax, CyMax], dtype= int).reshape(1, -1)[0])
        cv.circle(frame, Xcmax, 10, (0, 0, 255), -1)
        
        extBot = 0 

        if  maxA >= 0.9 * areaSUM:
            
            if not not np.shape(c):
                # determine the most extreme points along the contour
                argmaxdist = np.argmax(np.matmul(np.array(np.power((self.cmax - Xc),2) ), [1,1]))
                self.extBot2 =  tuple(self.cmax[argmaxdist,0])
                cv.circle(frame, self.extBot2, 10, (0, 255, 255), -1)
                cv.arrowedLine(frame, self.extBot2, Xc, (0, 255, 0), 2, tipLength=0.2)
                self.setV(Xc, self.extBot2)

        cv.putText(frame, 'RoboFish', (50, 50) , cv.FONT_HERSHEY_SIMPLEX ,1,(255, 0, 0) , 2, cv.LINE_AA) 

        cv.drawContours(frame, cnts, -1, (255 ,0, 0), 2)
        cv.imshow("Arrow", frame)
        
        key = cv.waitKey(30)
        if key == 'q' or key == 27: return True

        return False

    def setV(self , Cm , Cr):

        slope =  np.array(Cm) - np.array(Cr)  
        theta = np.arctan2(slope[1],slope[0])*-1
        fishPose = np.array([Cm[0], Cm[1]])
        dist = np.matmul(np.power(self.circle.T -fishPose,2) ,(1,1))
        fishRigion = np.argmin(dist)
        print fishRigion


        if fishRigion ==  0: tl, tr = -90 , 0
        if fishRigion ==  1: tl, tr = -135 , -45
        if fishRigion ==  2: tl, tr =  -90 ,180
        if fishRigion ==  3: tl, tr = -135 , 135
        if fishRigion ==  4: tl, tr = 90 , 180
        if fishRigion ==  5: tl, tr = 45 , 135
        if fishRigion ==  6: tl , tr= 0 , 90 
        if fishRigion ==  7: tl, tr = -45 , 45
        


        if np.rad2deg(theta) <=  tr and np.rad2deg(theta) >=  tl :
            u = np.cos(theta)*self.V
            v = np.sin(theta)*self.V
            print u ,v
            self.twist.linear.x = v; self.twist.linear.y = -u ;self. twist.linear.z = self.theta
            self.pub.publish(self.twist)

    def collaborate_im(self):


        cv.namedWindow('image')

        ilowH = 0
        ihighH = 179

        ilowS = 0
        ihighS = 255
        ilowV = 0
        ihighV = 255

        # create trackbars for color change
        cv.createTrackbar('lowH','image',ilowH,179, self.callback)
        cv.createTrackbar('highH','image',ihighH,179, self.callback)

        cv.createTrackbar('lowS','image',ilowS,255, self.callback)
        cv.createTrackbar('highS','image',ihighS,255, self.callback)

        cv.createTrackbar('lowV','image',ilowV,255, self.callback)
        cv.createTrackbar('highV','image',ihighV,255, self.callback)

        while(True):
            
            # grab the frame
            frame = self.im

            # get trackbar positions
            ilowH = cv.getTrackbarPos('lowH', 'image')
            ihighH = cv.getTrackbarPos('highH', 'image')
            ilowS = cv.getTrackbarPos('lowS', 'image')
            ihighS = cv.getTrackbarPos('highS', 'image')
            ilowV = cv.getTrackbarPos('lowV', 'image')
            ihighV = cv.getTrackbarPos('highV', 'image')

            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            lower_hsv = np.array([ilowH, ilowS, ilowV])
            higher_hsv = np.array([ihighH, ihighS, ihighV])
            mask = cv.inRange(hsv, lower_hsv, higher_hsv)

            frame = cv.bitwise_and(frame, frame, mask=mask)

            # show thresholded image
            cv.imshow('image', frame)
            k = cv.waitKey(1000) & 0xFF # large wait time to remove freezing
            if k == 113 or k == 27:
                break

    def callback(x):
        pass

if __name__ == '__main__':

    print ("Node starts")
    PFtry = fishBot() 
    rospy.spin()
