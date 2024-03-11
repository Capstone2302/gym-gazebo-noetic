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
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest
from sensor_msgs.msg import Image
import csv
from rosgraph_msgs.msg import Clock
from datetime import datetime

class GazeboWheelv1Env(gazebo_env.GazeboEnv):
    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "/home/mackenzie/gym-gazebo-noetic/gym_gazebo/envs/ros_ws/src/wheel_gazebo/launch/urdf.launch")

        # Define end conditions TODO
        # self.theta_threshold_radians = 12 * 2 * math.pi / 360
        self.x_threshold = 0.5 # when when x is farther than THRESHOLD pixels from the center_pixel, reset
        self.y_threshold = 450 # when we is greater than this reset
        self.center_pixel = 399
        self.vel_threshold = 30
        self.zeroed = np.pi*((1/4)-(1/32)+(1/128))
        self.n_actions = 1 #should be odd number 
        self.bridge = CvBridge()
        self.record = None

        # Logging telemetry
        self.do_telemetry = False
        if self.do_telemetry:
            self.csvFilename = datetime.now().strftime("%b%d-%H-%M-%S-rlwheel")
            self.csvfile = open('runs/telemetry/'+self.csvFilename+'.csv', 'w', newline = '')
            self.writer = csv.writer(self.csvfile)
            self.writer.writerow(['sim time', 'cam x','gazebo x', 'raw image received'])
            self.csvfile.close()

        # Setup pub/sub for state/action
        self.joint_pub = rospy.Publisher("/wheel/rev_position_controller/command", Float64, queue_size=1)
        self.wheel_sub = rospy.Subscriber('/wheel/joint_states', JointState, self.get_wheel_pos_callback)
        self.ball_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.get_ball_pos_callback)
        self.ball_sub_cam = rospy.Subscriber("/wheel/camera1/image_raw", Image, self.get_ball_pos_camera_callback, queue_size=1)
        self.sim_time_sub = rospy.Subscriber("/clock", Clock, self.get_sim_time)
        # Gazebo specific services to start/stop its behavior and
        # facilitate the overall RL environment
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.set_link = rospy.ServiceProxy('/gazebo/set_link_state', 
                                           SetLinkState)
        rospy.wait_for_service('/gazebo/set_link_state')

        # logging Video
        self.frameNumber = 0
        self.max_frames = 300

        # Setup the environment TODO
        self._seed()
        self.action_space = spaces.Discrete(self.n_actions) # output degrees 

        #TODO add dimension for wheel position which is needed for non-circular wheels=
        # low  = np.array([-self.x_threshold, -self.vel_threshold])
        # high = np.array([ self.x_threshold, self.vel_threshold, np.finfo(np.float32).max])
        # high = np.array([ self.x_threshold, self.x_threshold, 1])
        high = np.array([self.x_threshold, 1, np.finfo(np.float32).max,2*np.pi, np.finfo(np.float32).max]) # ball position x, ball position y, ball velocity, wheel position, wheel velocity
        low = np.array([-self.x_threshold, 0, -np.finfo(np.float32).max,-2*np.pi, -np.finfo(np.float32).max])
        self.observation_space = spaces.Box(low=low, high = high)

        # State data:
        self.ball_pos_x = None
        self.ball_pos_y = 0
        self.wheel_pos = None
        self.wheel_vel = None
        self.ball_vel = None
        self.prev_time = -1
        self.x_prev = 0
        self.y_prev = 0
        self.ball_pos_x_camera = -9999999

    def setRecordingState(self, record):
        self.record=record

    def get_wheel_pos_callback(self, msg):
        self.wheel_pos = msg.position[0]%(2*np.pi) - self.zeroed
        self.wheel_vel = msg.velocity[0]

    def get_sim_time(self, data):
        self.time = data.clock.secs + data.clock.nsecs/(1e9)

    def get_ball_pos_callback(self, msg):

        self.ball_pos_x = msg.pose[1].position.x
        self.ball_pos_y = msg.pose[1].position.z
    
        if self.do_telemetry:
            self.csvfile = open('runs/telemetry/'+self.csvFilename+'.csv', 'a')
            self.writer = csv.writer(self.csvfile)
            self.writer.writerow([str(self.time), "", str(self.ball_pos_x), ""])
            self.csvfile.close()

    def get_ball_pos_camera_callback(self, img_msg):
        self.raw_image = img_msg

        if self.do_telemetry:
            self.csvfile = open('runs/telemetry/'+self.csvFilename+'.csv', 'a')
            self.writer = csv.writer(self.csvfile)
            self.writer.writerow([str(self.time), "", "", "1"]) # magic number is conversion factor from pixels to meters, derivation on page 44 of Sean Ghaeli's logbook.
            self.csvfile.close()

    def process_img(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        output = cv_image.copy()
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 2,20, 
                                   param1=50,
                                   param2=30,
                                   minRadius=0,
                                   maxRadius=15)
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                self.ball_pos_x_camera = x - self.center_pixel #neg ball_pos means left of centre, pos = right of center
                self.ball_pos_y_camera = y
                cv2.circle(output, (x, y), r, (255, 0, 0), 4)               
                # if self.ball_pos_y > 450:
                #     self.reset_ball_pos()
                
                # self.PID_control()
            if len(circles) == 0:
                print("BALL MISSED WHERE BALL?!?!")
        # print("Camera ball position: " + str(self.ball_pos_x_camera))
                
        if self.do_telemetry:
            self.csvfile = open('runs/telemetry/'+self.csvFilename+'.csv', 'a')
            self.writer = csv.writer(self.csvfile)
            self.writer.writerow([str(self.time), str(self.ball_pos_x_camera*0.00209774908), "", ""]) # magic number is conversion factor from pixels to meters, derivation on page 44 of Sean Ghaeli's logbook.
            self.csvfile.close()
        # cv2.imshow("output", np.hstack([cv_image, output]))
        # cv2.imshow("Image window", output)

        # logging frames
        if self.record:
            output_dir = 'runs/video/images'
            image_path = os.path.join(output_dir, f"frame_{self.frameNumber:04d}.png")
            cv2.imwrite(image_path, output)

            # If the number of frames exceeds the maximum, delete the oldest ones
            if self.frameNumber >= self.max_frames:
                oldest_frame_path = os.path.join(output_dir, f"frame_{self.frameNumber - self.max_frames:04d}.png")
                os.remove(oldest_frame_path)

            self.frameNumber +=1

        # cv2.waitKey(1)
                
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        # Wait for data
        x_pos = None
        wheel_pos = None
        wheel_vel = None
        self.raw_image = None
        self.ball_pos_x = None

        action_msg = float(action)
        self.joint_pub.publish(action_msg)
        # print(self.time)
        last_time = self.time


        while (self.raw_image is None) or (self.ball_pos_x is None) or (self.wheel_vel is None):

            # Unpause simulation to make observations
            rospy.wait_for_service('/gazebo/unpause_physics')
            try:
                self.unpause()
            except (rospy.ServiceException) as e:
                print ("/gazebo/unpause_physics service call failed")

            try:
                self.raw_image = rospy.wait_for_message('/wheel/camera1/image_raw', Image, timeout=1)

                # Process data
                self.process_img(self.raw_image)
            except:
                print("failed image acquistion")
                pass

            # Pause
            rospy.wait_for_service('/gazebo/pause_physics')
            try:
                self.pause()
            except (rospy.ServiceException) as e:
                print ("/gazebo/pause_physics service call failed")
            
        x_pos = self.ball_pos_x

        t = self.time
        dt = t - last_time
        #self.prev_time = t

        dx = x_pos           - self.x_prev
        dy = self.ball_pos_y - self.y_prev

        self.ball_vel = round(dx/dt,2)

        # wheel_vel = round(wheel_vel, 2)

        # Define state  
        state = [x_pos, self.ball_pos_y, self.ball_vel, self.wheel_pos, self.wheel_vel/10]

        self.x_prev = x_pos
        self.y_prev = self.ball_pos_y
        
        # Check for end condition
        done = bool((self.ball_pos_y) < 0.2)
        
        reward = self.ball_pos_y
        # print('ball pos: ' , self.ball_pos_x)
        # print('reward ',reward)

        # Reset data
        self.ball_pos_x = None
        self.ball_pos_y= None
        self.wheel_pos = None
        self.wheel_vel = None
        self.raw_image = None
        return state, reward, done, {}
    
    def reset_ball_pos(self):    
        state_msg = ModelState()
        state_msg.model_name = 'ball'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.317
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
        print("**** RESETTING *****")

        # Reset world
        rospy.wait_for_service('/gazebo/set_link_state')
        # self.set_link(LinkState(link_name='wheel')) # WHY NO WORK
        
        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        self.joint_pub.publish(0)
        time.sleep(0.05)
        
        msg = SetModelConfigurationRequest()
        msg.model_name = 'wheel'
        msg.joint_names = ['rev']
        msg.joint_positions = [float(self.zeroed)]

        rospy.wait_for_service('/gazebo/set_model_configuration')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
            resp = set_state( msg )
        except rospy.ServiceException:
            print( "Service call failed")

        # Pause simulation
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
        
        self.reset_ball_pos()

        self.ball_pos_x = None
        self.raw_image = None
        while (self.raw_image is None) or (self.ball_pos_x is None):

            # Unpause simulation to make observations
            rospy.wait_for_service('/gazebo/unpause_physics')
            try:
                self.unpause()
            except (rospy.ServiceException) as e:
                print ("/gazebo/unpause_physics service call failed")

            try:
                self.raw_image = rospy.wait_for_message('/wheel/camera1/image_raw', Image, timeout=1)

                # Process data
                self.process_img(self.raw_image)
            except:
                print("failed image acquistion")
                pass

            # Pause
            rospy.wait_for_service('/gazebo/pause_physics')
            try:
                self.pause()
            except (rospy.ServiceException) as e:
                print ("/gazebo/pause_physics service call failed")
            
        state = [0, self.ball_pos_y, 0,0,0]
        self.x_prev = 0
        # Reset data
        self.ball_pos_x = None
        self.ball_pos_y= None
        self.wheel_pos = None
        self.wheel_vel = None
        self.raw_image = None

        # Process state
        return state
