#!/usr/bin/env python

# Team 1 members:
# Lemanov, Mack, Waiblinger, Weber, Wilke

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class Central:
    def __init__(self):
        # initialize class variables
        self.stiffness = False  
        pass

    def key_cb(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def image_cb(self,data):
        bridge_instance = CvBridge()
        try:
            cv_image = bridge_instance.imgmsg_to_cv2(data,"bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            rospy.logerr(e)
        
        filtered_image = cv2.inRange(hsv_image, (0, 255*60/100, 255*60/100), (15, 255, 255))
        self.detect_blob(filtered_image)
        cv2.waitKey(3) # a small wait time is needed for the image to be displayed correctly
    
    def detect_blob(self, image):
        params = cv2.SimpleBlobDetector_Params()

        # Filter by area size
        params.filterByArea = True
        params.minArea = 20

        # Filter by color in hsv space
        params.filterByColor = True
        params.blobColor = 255

        # Filter by pixel intensity
        params.minThreshold = 200
        params.maxThreshold = 255

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.01
        
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.01
        
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(image)
        blank = np.zeros((1, 1))
        blobs = cv2.drawKeypoints(image, keypoints, blank, (255, 0, 0),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
        if keypoints!=[]:
            point = Point(keypoints[0].pt[0], keypoints[0].pt[1], 0)
            rospy.loginfo("Detected Blob at: " + str(keypoints[0].pt))
            self.blobPub.publish(point)
          
        cv2.imshow("Blobs Using Area", blobs)
            

    # sets the stiffness for all joints. can be refined to only toggle single joints, set values between [0,1] etc
    def set_stiffness(self,value):
        if value == True:
            service_name = '/body_stiffness/enable'
        elif value == False:
            service_name = '/body_stiffness/disable'
        try:
            stiffness_service = rospy.ServiceProxy(service_name,Empty)
            stiffness_service()
        except rospy.ServiceException, e:
            rospy.logerr(e)

    def central_execute(self):
        rospy.init_node('central_node',anonymous=True) #initilizes node, sets name

        # create several topic subscribers
        rospy.Subscriber("key", String, self.key_cb)
        rospy.Subscriber("/nao_robot/camera/top/camera/image_raw",Image,self.image_cb)
        self.blobPub = rospy.Publisher("blob_position",Point,queue_size=10)

        self.set_stiffness(True) # always check that your robot is in a stable position before disabling the stiffness!!

        rate = rospy.Rate(10) # sets the sleep time to 10ms

        while not rospy.is_shutdown():
            rate.sleep()

if __name__=='__main__':
    # instantiate class and start loop function
    central_instance = Central()
    central_instance.central_execute()
