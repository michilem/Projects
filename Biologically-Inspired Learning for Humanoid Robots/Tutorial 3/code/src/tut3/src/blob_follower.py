#!/usr/bin/env python

# Team 1 members:
# Lemanov, Mack, Waiblinger, Weber, Wilke

# [HeadYaw, HeadPitch, LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw,
#   LHand, LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll, RHipYawPitch,
#   RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll, RShoulderPitch, RShoulderRoll,
#   RElbowYaw, RElbowRoll, RWristYaw, RHand]25

import rospy
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed,Bumper,HeadTouch
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from bilhr import get_joint_mlp

class Central():
    def __init__(self):
        # initialize class variables
        self.joint_names = []
        self.joint_angles = []
        self.joint_velocities = []
        self.jointPub = 0
        self.stiffness = True
        self.last_point = []
        self.network = get_joint_mlp()

        self.network.load_weights("/home/nao/ros/tut3/src/tut3/src/joint_parameters.npz")
        pass

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
        rospy.loginfo("touch button: "+str(data.button)+" state: "+str(data.state))
        if data.button == 2 and data.state == 1:
            pred = self.network.predict(self.last_point)

            pitch = np.clip(pred[0], -2, 2)
            roll = np.clip(pred[1], -0.3, 1.3)

            self.execute_motion("LShoulderPitch", pitch)
            self.execute_motion("LShoulderRoll", roll)

            print("Target pitch={:.2f}, roll={:.2f}".format(pitch, roll))
    

    def blob_cb(self, point):
        x = point.x * (1/320.0)
        y = point.y * (1/240.0)
        self.last_point = [x, y]
        pass


    def execute_motion(self, name, angle):
        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append(name) # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(angle) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)


    def central_execute(self):
        rospy.init_node('central_node',anonymous=True) #initilizes node, sets name

        # create several topic subscribers
        rospy.Subscriber("key", String, self.key_cb)
        rospy.Subscriber("joint_states",JointState,self.joints_cb)
        rospy.Subscriber("tactile_touch",HeadTouch,self.touch_cb)
        rospy.Subscriber("blob_position",Point,self.blob_cb)
        self.jointPub = rospy.Publisher("joint_angles",JointAnglesWithSpeed,queue_size=10)

        rate = rospy.Rate(1000) # sets the sleep time to 10ms

        while not rospy.is_shutdown():
            # if self.last_point != []:
            #     pred = self.network.predict(self.last_point)
            #     pitch = np.clip(pred[0], -2, 2)
            #     roll = np.clip(pred[1], -0.3, 1.3)
            #     self.execute_motion("LShoulderPitch", pitch)
            #     self.execute_motion("LShoulderRoll", roll)
            #     print("Target pitch={:.2f}, roll={:.2f}".format(pitch, roll))
            rate.sleep()

if __name__=='__main__':
    # instantiate class and start loop function
    central_instance = Central()
    central_instance.central_execute()
