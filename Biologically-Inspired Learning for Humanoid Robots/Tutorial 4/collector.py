#!/usr/bin/env python

# Team 1 members:
# Lemanov, Mack, Waiblinger, Weber, Wilke

# [HeadYaw, HeadPitch, LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw,
#   LHand, LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll, RHipYawPitch,
#   RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll, RShoulderPitch, RShoulderRoll,
#   RElbowYaw, RElbowRoll, RWristYaw, RHand, BlobX, BlobY]27

import rospy
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed,Bumper,HeadTouch
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

class Central:
    def __init__(self):
        # initialize class variables
        self.joint_names = []
        self.joint_angles = []
        self.joint_velocities = []
        self.jointPub = 0
        self.point = None  
        self.saved_points = []
        self.header = "HeadYaw, HeadPitch, LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw, LHand, LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll, RHipYawPitch, RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll, RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw, RHand, BlobX, BlobY"
        

    def key_cb(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


    def joints_cb(self,data):
        #rospy.loginfo("joint states "+str(data.name)+str(data.position))
        # store current joint information in class variables
        self.joint_names = data.name 
        self.joint_angles = data.position
        self.joint_velocities = data.velocity
        pass

    # buttons control left arm position (1: safe_pos, 2: wave_motion)
    def touch_cb(self,data):
        # react on button only on pressed down (not release)
        if data.button == 1 and data.state == 1:
            recording = list(self.joint_angles) + [self.point.x, self.point.y]
            self.saved_points.append(recording)
            rospy.loginfo(recording)
            
        elif data.button == 2:
            arr = np.asarray(self.saved_points)
            np.savetxt("bar.csv", arr, delimiter=',', header=self.header, comments="")

    def collect_cb(self, blob):
        rospy.loginfo("collecting " + str(blob))
        self.point = blob


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
        rospy.Subscriber("joint_states",JointState,self.joints_cb)
        rospy.Subscriber("blob_position",Point,self.collect_cb)
        rospy.Subscriber("tactile_touch",HeadTouch,self.touch_cb)

        rate = rospy.Rate(10) # sets the sleep time to 10ms

        while not rospy.is_shutdown():
            
            rate.sleep()

if __name__=='__main__':
    # instantiate class and start loop function
    central_instance = Central()
    central_instance.central_execute()
