import gym
import rospy
import roslaunch
import time
import numpy as np
from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from gym.utils import seeding
import copy
import math
import os

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
from gazebo_msgs.srv import SetLinkState
from gazebo_msgs.msg import LinkState
import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import Image


class GazeboWheelv1Env(gazebo_env.GazeboEnv):
    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "/home/mackenzie/gym-gazebo-noetic/gym_gazebo/envs/ros_ws/src/wheel_gazebo/launch/urdf.launch")

        # Define end conditions TODO
        # self.theta_threshold_radians = 12 * 2 * math.pi / 360
        self.x_threshold = 0.2 # when when x is farther than lsdkfj pixels from the center_pixel, reset
        self.y_threshold = 450 # when we is greater than this reset
        self.center_pixel = 399
        self.vel_threshold = 30
        self.n_actions = 3 #should be odd number 
        self.bridge = CvBridge()

        # Setup pub/sub for state/action
        self.joint_pub = rospy.Publisher("/wheel/rev_position_controller/command", Float64, queue_size=1)
        self.wheel_sub = rospy.Subscriber('/wheel/joint_states', JointState, self.get_wheel_pos_callback)
        self.ball_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.get_ball_pos_callback)

        # Gazebo specific services to start/stop its behavior and
        # facilitate the overall RL environment
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.set_link = rospy.ServiceProxy('/gazebo/set_link_state', 
                                           SetLinkState)
        rospy.wait_for_service('/gazebo/set_link_state')

        # Setup the environment TODO
        self._seed()
        self.action_space = spaces.Discrete(self.n_actions) # output degrees 
        # cartesian product, 3 Dimensions - ball_pos_x, ball_pos_y, wheel_pos degree

        #TODO add dimension for wheel position which is needed for non-circular wheels=
        low  = np.array([-self.x_threshold, -self.vel_threshold])
        high = np.array([ self.x_threshold, self.vel_threshold, np.finfo(np.float32).max])
        self.observation_space = spaces.Box(low=-high, high = high)

        # State data:
        self.ball_pos_x = None
        self.ball_pos_y = 0
        self.wheel_pos = None
        self.wheel_vel = None
        self.ball_vel = None
        self.prev_time = -1
        self.x_prev = 0
        self.y_prev = 0

    def get_wheel_pos_callback(self, msg):
        # msg_str = str(msg)
        # i = msg_str.find('position: [')+11
        # msg_str = msg_str[i:]
        # i = msg_str.find(']')
        # self.wheel_pos = float(msg_str[0:i])

        msg_str = str(msg)
        i = msg_str.find('velocity: [')+11
        msg_str = msg_str[i:]
        i = msg_str.find(']')
        self.wheel_vel = float(msg_str[0:i])
        
        # print('wheel pos read: '+ str(self.wheel_pos))
        # self.wheel_pos_write = self.wheel_pos+1
        # self.joint_pub.publish(self.wheel_pos_write)
        # print('position published: '+ str(self.wheel_pos+1))

    def get_ball_pos_callback(self, msg):
        self.ball_pos_x = msg.pose[1].position.x
        self.ball_pos_y = msg.pose[1].position.z
        
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        # Wait for data
        x_pos = None
        wheel_pos = None

        # Unpause simulation to make observations
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        # timeout = time.time() + 5
        # diff = time.time()
        while x_pos is None or wheel_vel is None:
            x_pos = self.ball_pos_x
            # wheel_pos = self.wheel_pos
            wheel_vel = (self.wheel_vel)
            # if time.time() > timeout:
            #     self.reset_ball_pos()
        # diff = time.time()-diff
        # print('end ', diff*1000, ' ms')
        
        # Pause
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        t = rospy.get_rostime().nsecs/10**6
        # print('Curr time: ' + str(t))
        dt = t - self.prev_time
        self.prev_time = t

        dx = x_pos           - self.x_prev
        dy = self.ball_pos_y - self.y_prev
        self.x_prev = x_pos
        self.y_prev = self.ball_pos_y

        ball_vel = 0
        # if dt != 0:
        ball_vel = round(dx/dt*10**5,2)

        wheel_vel = round(wheel_vel, 2)
        print('x_pos: '+ str(x_pos))
        # print('ball pos y read: '+ str(self.ball_pos_y))
        # print('wheel pos read: '+ str(self.wheel_pos))
        # print('ball_vel: ' + str(ball_vel))
        # print('wheel_vel: '+ str(wheel_vel))


        # Take action        
        action = action - (self.n_actions-1)/2
        self.wheel_vel += action*0.2
        # if action == 0:
        #     self.wheel_vel -= 0.2
        # else:
        #     self.wheel_vel += 0.2

        # action_msg = float(self.wheel_pos)
        action_msg = float(self.wheel_vel)
        self.joint_pub.publish(action_msg)
        print('action: '+ str(int(action)))
        # print('wheel pos write: '+ str(action_msg))
        # print('wheel vel write: '+ str(action_msg))

        # Define state  
        state = [x_pos, wheel_vel, ball_vel]

        # Check for end condition
        done = (abs(self.ball_pos_x) > self.x_threshold)
        # done = self.ball_pos_y > 500 
        done = bool(done)
        # print('isDone: '+ str((done)))

        if not done:
            reward = 1.0
        else:
            reward = 0

        # Reset data
        self.ball_pos_x = None
        self.wheel_pos = None
        self.wheel_pos = None
        return state, reward, done, {}
    
    def reset_ball_pos(self):    
        state_msg = ModelState()
        state_msg.model_name = 'ball'
        state_msg.pose.position.x = np.random.randint(-100,100)/4000
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.39
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0
        rospy.wait_for_service('/gazebo/set_model_state')
        # print('ball reset')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
        except rospy.ServiceException:
            print( "Service call failed")
    
    def reset(self): 
        # Reset world
        rospy.wait_for_service('/gazebo/set_link_state')
        # self.set_link(LinkState(link_name='wheel')) # WHY NO WORK
        self.joint_pub.publish(float(0)) #vel
        
        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        time.sleep(0.5)
        # Pause simulation
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
        
        self.reset_ball_pos()


        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        # Wait for data
        x_pos = None
        wheel_pos = None
        timeout = time.time() + 5

        while x_pos is None or wheel_vel is None:
            x_pos = self.ball_pos_x
            # wheel_pos = self.wheel_pos
            wheel_vel = (self.wheel_vel)
            # if time.time() > timeout:
            #     self.reset_ball_pos()

        # comment out if vel control
        # action_msg = float(self.wheel_pos)
        # self.joint_pub.publish(action_msg)
        # print('wheel pos published: '+ str(action_msg))
        #### 

        wheel_vel = round(wheel_vel,2)
        state = [x_pos, wheel_vel,0]
        
        # Pause simulation
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
        
        # Reset data
        self.ball_pos_x = None
        self.wheel_pos = None
        self.wheel_vel = None

        # Process state
        return state