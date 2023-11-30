#!/usr/bin/env python

# Team 1 members:
# Lemanov, Mack, Waiblinger, Weber, Wilke

# [HeadYaw, HeadPitch, LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw,
#   LHand, LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll, RHipYawPitch,
#   RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll, RShoulderPitch, RShoulderRoll,
#   RElbowYaw, RElbowRoll, RWristYaw, RHand]25

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed,Bumper,HeadTouch
from sensor_msgs.msg import JointState

class Central:
    def __init__(self):
        # initialize class variables
        self.joint_names = []
        self.joint_angles = []
        self.joint_velocities = []
        self.jointPub = 0
        self.stiffness = True  
        self.save_pos_reached = False
        self.toggle_motion = False
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

    # bumpers control stiffness (left: True, right: False)
    def bumper_cb(self,data):
        rospy.loginfo("bumper: "+str(data.bumper)+" state: "+str(data.state))
        if data.bumper == 0:
            self.stiffness = True
        elif data.bumper == 1:
            self.stiffness = False

    # buttons control left arm position (1: safe_pos, 2: wave_motion)
    def touch_cb(self,data):
        rospy.loginfo("touch button: "+str(data.button)+" state: "+str(data.state))
        # react on button only on pressed down (not release)
        if data.button == 1 and data.state == 1:
            self.toggle_motion = False
            self.safe_position_left_arm()
            self.save_pos_reached = True
        
        # wave only when arm is in safe position
        if data.button == 2 and data.state == 1:
            if self.save_pos_reached:
                self.toggle_motion = True
                self.save_pos_reached = False
            else:
                self.toggle_motion = False

            rospy.loginfo("Toggle Motion" + str(self.toggle_motion))

    def safe_position_left_arm(self):
        self.execute_motion("LShoulderPitch", 1.72)
        self.execute_motion("LShoulderRoll", 0.06)
        self.execute_motion("LElbowYaw", -1.98)
        self.execute_motion("LElbowRoll", -0.17)

    def position_one_left_arm(self):
        self.execute_motion("LShoulderPitch", 0.32)
        self.execute_motion("LShoulderRoll", -0.15)
        self.execute_motion("LElbowYaw", -1.14)
        self.execute_motion("LElbowRoll", -1.51)

    def position_two_left_arm(self):
        self.execute_motion("LShoulderPitch", 0.53)
        self.execute_motion("LShoulderRoll", 0.64)
        self.execute_motion("LElbowYaw", -2.10)
        self.execute_motion("LElbowRoll", -1.34)

    def execute_motion(self, name, angle):
        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append(name) # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(angle) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)

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
        rospy.Subscriber("bumper",Bumper,self.bumper_cb)
        rospy.Subscriber("tactile_touch",HeadTouch,self.touch_cb)
        self.jointPub = rospy.Publisher("joint_angles",JointAnglesWithSpeed,queue_size=10)

        self.set_stiffness(True) # always check that your robot is in a stable position before disabling the stiffness!!

        rate = rospy.Rate(10) # sets the sleep time to 10ms

        while not rospy.is_shutdown():
            self.set_stiffness(self.stiffness)
            if self.toggle_motion:
                self.save_pos_reached = False
                self.position_one_left_arm()
                rospy.sleep(2)
                self.position_two_left_arm()
                rospy.sleep(2)
                rospy.loginfo("Toggle Motion down" + str(self.toggle_motion))
            rate.sleep()

if __name__=='__main__':
    # instantiate class and start loop function
    central_instance = Central()
    central_instance.central_execute()
