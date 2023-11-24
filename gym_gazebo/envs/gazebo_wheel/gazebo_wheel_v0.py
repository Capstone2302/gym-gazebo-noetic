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
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import Image


class GazeboWheelv0Env(gazebo_env.GazeboEnv):
    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "/home/mackenzie/enph353_gym-gazebo-noetic/gym_gazebo/envs/ros_ws/src/wheel_gazebo/launch/urdf.launch")

        # Define end conditions TODO
        # self.theta_threshold_radians = 12 * 2 * math.pi / 360
        self.x_threshold = 50 # when when x is farther than lsdkfj pixels from the center_pixel, reset
        self.y_threshold = 450 # when we is greater than this reset
        self.center_pixel = 399
        self.bridge = CvBridge()

        # Setup pub/sub for state/action
        self.joint_pub = rospy.Publisher("/wheel/rev_position_controller/command", Float64, queue_size=1)
        self.wheel_sub = rospy.Subscriber('/wheel/joint_states', JointState, self.get_wheel_pos_callback)
        self.ball_sub = rospy.Subscriber("/wheel/camera1/image_raw", Image, self.get_ball_pos_callback)

        # Gazebo specific services to start/stop its behavior and
        # facilitate the overall RL environment
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.set_link = rospy.ServiceProxy('/gazebo/set_link_state', 
                                           SetLinkState)
        rospy.wait_for_service('/gazebo/set_link_state')

        # Setup the environment TODO
        self._seed()
        self.action_space = spaces.Discrete(2) # output degrees 
        # cartesian product, 3 Dimensions - ball_pos_x, ball_pos_y, wheel_pos degree

        #TODO add dimension for wheel position which is needed for non-circular wheels=
        low  = np.array([-self.x_threshold])
        high = np.array([ self.x_threshold])
        self.observation_space = spaces.Box(low=low, high = high)

        # State data:
        self.ball_pos_x = None
        self.ball_pos_y = 0
        self.wheel_pos = None
        self.wheel_vel = None

    def get_wheel_pos_callback(self, msg):
        msg_str = str(msg)
        i = msg_str.find('position: [')+11
        msg_str = msg_str[i:]
        i = msg_str.find(']')
        self.wheel_pos = float(msg_str[0:i])

        msg_str = str(msg)
        i = msg_str.find('velocity: [')+11
        msg_str = msg_str[i:]
        i = msg_str.find(']')
        self.wheel_vel = float(msg_str[0:i])
        
        # print('wheel pos read: '+ str(self.wheel_pos))
        # self.wheel_pos_write = self.wheel_pos+1
        # self.joint_pub.publish(self.wheel_pos_write)
        # print('position published: '+ str(self.wheel_pos+1))

    def get_ball_pos_callback(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        gray = cv2.cvtColor(cv_image, cv2
                            .COLOR_BGR2GRAY)
        output = cv_image.copy()
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 2,20, 
                                   param1=50,
                                   param2=30,
                                   minRadius=0,
                                   maxRadius=15)
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            if (len(circles)) > 1:
                print("######### TOO MANY CIRCLES #######")
            for (x, y, r) in circles:
                cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                # cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                # print(str(y))
                self.ball_pos_x = int((x - self.center_pixel)) #neg ball_pos means left of centre, pos = right of center
                self.ball_pos_y = y
                # print('ball pos x read: '+ str(self.ball_pos_x))
                # print('ball pos y read: '+ str(self.ball_pos_y))
                # if self.ball_pos_y >450:
                #     self.reset_ball_pos()
                if abs(self.ball_pos_x) > self.x_threshold:
                    cv2.circle(output, (x, y), r, (255, 0, 0), 4)
        else:
            self.ball_pos_x=100
            self.ball_pos_y=100
            print("######################## BALL NOT DETECTED ##################")

        # cv2.imshow("output", np.hstack([cv_image, output]))
        cv2.imshow("Image window", output)
        cv2.waitKey(1)
        
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
        while x_pos is None or wheel_pos is None:
            x_pos = self.ball_pos_x
            wheel_pos = self.wheel_pos
            wheel_vel = self.wheel_vel
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

        print('ball pos x read: '+ str(self.ball_pos_x))
        # print('ball pos y read: '+ str(self.ball_pos_y))
        # print('wheel pos read: '+ str(self.wheel_pos))
        print('wheel_vel: '+ str(self.wheel_vel))


        # Take action        
        # action = action - 5
        # self.wheel_pos += action*0.2
        if action == 0:
            self.wheel_vel += 0.2
        else:
            self.wheel_vel -= 0.2
            

        # action_msg = float(self.wheel_pos)
        action_msg = float(self.wheel_vel)
        self.joint_pub.publish(action_msg)
        print('action: '+ str(int(action)))
        # print('wheel pos write: '+ str(action_msg))
        print('wheel vel write: '+ str(action_msg))

        # Define state  
        state = [x_pos]

        # Check for end condition
        done = abs(self.ball_pos_x) > self.x_threshold
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
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.5
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0
        rospy.wait_for_service('/gazebo/set_model_state')
        print('ball reset')
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
        time.sleep(0.1)
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

        while x_pos is None or wheel_pos is None:
            x_pos = self.ball_pos_x
            wheel_pos = self.wheel_pos
            wheel_vel = self.wheel_vel
            # if time.time() > timeout:
            #     self.reset_ball_pos()

        # comment out if vel control
        # action_msg = float(self.wheel_pos)
        # self.joint_pub.publish(action_msg)
        # print('wheel pos published: '+ str(action_msg))
        #### 

        state = [x_pos]
        
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