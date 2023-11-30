#!/usr/bin/env python

# Team 1 members:
# Lemanov, Mack, Waiblinger, Weber, Wilke

# [HeadYaw, HeadPitch, LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll,
#  LWristYaw, LHand, LHipYawPitch, LHipRoll, LHipPitch, LKneePitch,
#  LAnklePitch, LAnkleRoll, RHipYawPitch, RHipRoll, RHipPitch, RKneePitch,
#  RAnklePitch, RAnkleRoll, RShoulderPitch, RShoulderRoll, RElbowYaw,
#  RElbowRoll, RWristYaw, RHand]


import rospy
import numpy as np
from std_srvs.srv import Empty
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from sklearn.tree import DecisionTreeRegressor
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed, Bumper, HeadTouch
import pickle

class Central():
    # setup this node
    def __init__(self):
        # necessary joint values to perform movements
        self.joint_names = []
        self.joint_angles = []
        self.joint_velocities = []
        self.jointPub = 0

        self.GAMMA = 1
        self.RMAX = 20

        # initial joint angles for kick position
        self.initial_position = [
            -0.023, 0.153, 1.478, 0.430, -1.305, -0.190, 0.116, 0.126, -0.003,
            0.2, # LHipRoll
            0.027, 0.185, 0.041, 0.030, -0.003, -0.096, -0.171, -0.092, 0.085,
            -0.213, 1.501, -0.161, 2.085, 0.345, 1.016, 0.360
        ]

        # S = {hip angles x goalie position}
        # A = {actions the agent can choose from}
        self.goalie_positions = [0, 1, 2] #['l', 'm', 'r']
        self.hip_positions = [-0.3, -0.05, 0.2, 0.45, 0.7, 10.0]
        self.actions = [0, 1, 2] #['in', 'out', 'go']

        # transition function (lookup table)
        self.P = np.ones((6, 3))
        self.P[0, 0] = 0
        self.P[4, 1] = 0

        # predicts reward from state and action
        self.reward_tree = DecisionTreeRegressor()
        self.train_data = {}
        self.policy = np.zeros((3, 6, 3))
        self.visits = np.zeros((3, 6))

        self.reward_list = []

        self.reset()


    # move all joints into the initial position
    def safe_position(self):
        for name, pos in zip(self.joint_names, self.initial_position):
            self.move_joint(name, pos)
    

    # reset all values except learned reward function
    def reset(self):
        self.state = 2 # index in hip_positions
        self.goalie = 1
        self.reward = 0
        self.min_visits = 0
        self.explore = False

        self.reward_list.append(0)
        self.looking_for_goalie = True
        self.choose_next_action = False
        self.wait_for_reward = False

        self.set_stiffness(True)
        self.safe_position()


    # execute chosen action
    def take_action(self, action):
        self.choose_next_action = False

        if action == 2:
            self.kick()
            self.wait_for_reward = True
        else:
            index = self.get_next_state(self.state, action)
            self.reward = -5 if index == self.state else -1
            self.move_joint('LHipRoll', self.hip_positions[index])


    # compute index of the next state after using action a
    def get_next_state(self, state, action):
            if state == 5: # cannot leave terminal state
                return 5
            
            if action == 0: # move leg inwards
                if state == 0: # leg already all the way in
                    return state
                else:
                    return state-1

            elif action == 1: # move leg outwards
                if state == 4: # leg already all the way out
                    return state
                else:
                    return state+1

            else: # kick
                return 5


    # change pitch angle of the left hip (fast)
    def kick(self):
        self.move_joint('LHipPitch', -1.188, speed=0.35)


    # update reward function model
    def update_tree(self, state, action, reward):
        entry = (self.goalie, state, action)

        self.train_data[entry] = reward

        samples = np.array(self.train_data.keys())
        targets = np.array(self.train_data.values())

        self.reward_tree.fit(samples, targets)

        return True


    # decide between exploration and exploitation
    def check_policy(self, state, action):
        rospy.loginfo("Max Model Reward: " + str(self.policy[self.goalie, state, action]))
        
        return self.policy[self.goalie, state, action] < 0.4 * self.RMAX


    # update policy
    def value_iteration(self, explore):
        self.min_visits = np.min(self.visits)
        self.explore = explore

        for s in range(len(self.hip_positions)):
            for a in range(len(self.actions)):
                self.policy[self.goalie, s, a] = self.Q(s, a)


    # reward function
    def R(self, state, action):
        inputs = np.array([[self.goalie, state, action]])

        return self.reward_tree.predict(inputs)


    # action value function
    def Q(self, state, action, horizon=10):
        next_state = self.get_next_state(state, action)

        # stop recursion
        horizon -= 1
        if horizon == 0 or state == 5: return 0

        # compute best action recursively
        max_q = -10000000
        for a in self.actions:
            q = self.Q(next_state, a, horizon)
            if max_q < q: max_q = q
           
        
        if self.explore and self.visits[self.goalie, state] == self.min_visits:
            r = self.RMAX
        else:
            r = self.R(state, action)
        pq = self.P[state, action] * max_q

        return r + self.GAMMA * pq


    # main loop
    def central_execute(self):
        # initilizes node, sets name
        rospy.init_node('central_node', anonymous=True)

        # create several topic subscribers
        rospy.Subscriber("joint_states", JointState, self.joints_cb)
        rospy.Subscriber("bumper", Bumper, self.bumper_cb)
        rospy.Subscriber("tactile_touch", HeadTouch, self.touch_cb)
        rospy.Subscriber("blob_position", Point, self.detector_cb)
        self.jointPub = rospy.Publisher("joint_angles", JointAnglesWithSpeed, queue_size=10)

        rate = rospy.Rate(10) # sets the sleep time to 10ms

        action = None

        while not rospy.is_shutdown():
            if self.looking_for_goalie:
                rate.sleep()
                continue
            
            if self.choose_next_action:
                # a <- argmax Q(s, a')
                action = np.argmax(self.policy[self.goalie, self.state, :])

                rospy.loginfo("Chose action "+str(action))
                try:
                    rospy.loginfo("Predicted Reward: " + str(self.reward_tree.predict(np.array([[self.goalie, self.state, action]]))))
                except:
                    print("Tree not fit")

                # execute a
                self.take_action(action) #sets self.choose_next_action = False 

            # obtain reward
            if self.wait_for_reward:
                rate.sleep()
                continue
            
            rospy.loginfo("Got reward "+str(self.reward))
            self.reward_list[-1] += self.reward

            # observe state s'
            next_state = self.get_next_state(self.state, action)

            # increment visits(s, a)
            self.visits[self.goalie, self.state] += 1

            # (Pm, Rm, CH) <- UpdateModel(s, a, r, s', S, A)
            model_changed = self.update_tree(self.state, action, self.reward)

            # exp <- CheckPolicy(Pm, Rm)
            explore = self.check_policy(self.state, action)

            # if CH: ComputeValues(Rmax, Pm, Rm, Sm, A, exp)
            if model_changed: self.value_iteration(explore)

            rospy.loginfo("Exploration "+str(explore))

            # s <- s'
            self.state = next_state
            if self.state != 5:
                self.choose_next_action = True
            else:
                self.reset()

            rate.sleep()
        
        # save rewards, policy and dtr on node shutdown
        rew = np.asarray(self.reward_list)
        np.savetxt("bar.csv", rew, delimiter=',')

        pol = np.asarray(self.policy)
        np.savez_compressed("policy", pol)

        # TODO
        pickle.dumps(self.reward_tree)


    # --- callbacks ----------------------------------------------------------
    def joints_cb(self, data):
        # store current joint information in class variables
        self.joint_names = data.name
        self.joint_angles = data.position
        self.joint_velocities = data.velocity

    def set_stiffness(self, value):
        # sets the stiffness for all joints [True, False]
        service_name = '/body_stiffness/enable' if value else '/body_stiffness/disable'

        try: rospy.ServiceProxy(service_name,Empty)()
        except rospy.ServiceException as e: rospy.logerr(e)

    def move_joint(self, name, angle, speed=0.1):
        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append(name)
        joint_angles_to_set.joint_angles.append(angle) # order like names!!
        joint_angles_to_set.relative = False # increment position?
        joint_angles_to_set.speed = speed
        self.jointPub.publish(joint_angles_to_set)

    def touch_cb(self, data):
        rospy.loginfo("Button: {} State: {}".format(data.button, data.state))
        if data.state == 1: return # avoid activating twice

        # minor punishment for moving the leg
        if data.button == 1: self.reset()
        
        # major punishment for missing the goal
        elif data.button == 2: self.reward = -20

        # major reward for scoring a goal
        elif data.button == 3: self.reward = 20

        self.wait_for_reward = False

    def bumper_cb(self, data):
        rospy.loginfo("Bumper: {} State: {}".format(data.bumper, data.state))
        if data.state == 1: return # avoid activating twice

        # right bumper starts decision process
        if data.bumper == 0:
            rospy.loginfo("Goalie Position: " + str(self.goalie))
            self.looking_for_goalie = False
            self.choose_next_action = True

    def detector_cb(self, point):
        if not self.looking_for_goalie: return
        
        l_threshold = 130
        r_threshold = 170

        if point.x < l_threshold: # goalie is left
            index = 0
        elif point.x > r_threshold: # goalie is right
            index = 2
        else: # goalie is middle
            index = 1

        self.goalie = index


if __name__ == '__main__':
    node = Central()
    node.central_execute()
