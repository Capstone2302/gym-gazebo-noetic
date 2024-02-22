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
import csv
from rosgraph_msgs.msg import Clock
from datetime import datetime

class GazeboWheelv1Env(gazebo_env.GazeboEnv):
    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "/home/seanghaeli/gym-gazebo-noetic/gym_gazebo/envs/ros_ws/src/wheel_gazebo/launch/urdf.launch")

        # Define end conditions
        # self.theta_threshold_radians = 12 * 2 * math.pi / 360
        self.x_coord_limit = 0.1524 # radius of wheel
        self.ball_vel_threshold = 30
        self.y_coord_reset_threshold = 0.1 # when y is greater than this reset
        self.bridge = CvBridge()
        self.record = None
        self.ball_pos_gazebo_time = 0

        # Logging telemetry
        self.do_telemetry = True
        self.csvFilename = datetime.now().strftime("%b%d-%H-%M-%S-rlwheel")
        if self.do_telemetry:
            self.csvfile = open('runs/telemetry/'+self.csvFilename+'.csv', 'w', newline = '')
            self.writer = csv.writer(self.csvfile)
            self.writer.writerow(['sim time', 'cam x','gazebo x', 'raw image received'])
            self.csvfile.close()

            self.csvfile = open('runs/telemetry/'+self.csvFilename+'_callback_times.csv', 'a')
            self.writer = csv.writer(self.csvfile)
            self.writer.writerow(['Wheel Update Time', 'Ball Update Time', 'Time Update Time'])
            self.csvfile.close()
        
        self.counter = 0
        self.max_array_length = 3000
        self.wheel_update_times = [0] * self.max_array_length
        self.ball_update_times = [0] * self.max_array_length
        self.time_update_times = [0] * self.max_array_length

        # Setup pub/sub for state/action
        self.joint_pub = rospy.Publisher("/wheel/rev_effort_controller/command", Float64, queue_size=1)
        self.wheel_sub = rospy.Subscriber('/wheel/joint_states', JointState, self.get_wheel_pos_callback, queue_size=1)
        self.ball_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.get_ball_pos_callback, queue_size=1)
        # self.ball_sub_cam = rospy.Subscriber("/wheel/camera1/image_raw", Image, self.get_ball_pos_camera_callback, queue_size=1)
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
        self.action_space = spaces.Box(low=-1, high=1, shape=(1,), dtype=np.float32) # output torque continuous action space 
        # cartesian product, 3 Dimensions - ball_pos_x, ball_pos_y, wheel_pos degree

        obs_space_magnitude  = np.array([self.x_coord_limit, self.ball_vel_threshold]) # ball position and ball speed
        self.observation_space = spaces.Box(low=-obs_space_magnitude, high = obs_space_magnitude)

        # State data:
        self.ball_pos_x = None
        self.ball_pos_y = None
        self.wheel_pos = None
        self.wheel_vel = None
        self.ball_vel = None
        self.prev_time = 0
        self.prev_ball_pos = 0
        self.x_prev = 0
        self.y_prev = 0
        self.ball_pos_x_camera = -9999999

    def setRecordingState(self, record):
        self.record=record

    def get_wheel_pos_callback(self, msg):
        self.wheel_vel = msg.velocity[0]

        if self.counter < self.max_array_length:
            self.wheel_update_times[self.counter] = self.time
            self.counter += 1
        self.check_and_save()

    def get_sim_time(self, data):
        self.time = data.clock.secs + data.clock.nsecs/(1e9)
        if self.counter < self.max_array_length:
            self.time_update_times[self.counter] = self.time
            self.counter += 1
        self.check_and_save()

    def get_ball_pos_callback(self, msg):

        # if self.time - self.ball_pos_gazebo_time >= 35e-3:
        #     self.ball_pos_gazebo_time = self.time
        self.ball_pos_x = msg.pose[1].position.x
        self.ball_pos_y = msg.pose[1].position.z
        if self.counter < self.max_array_length:
            self.ball_update_times[self.counter] = self.time
            self.counter += 1
        self.check_and_save()
    
        if self.do_telemetry:
            self.csvfile = open('runs/telemetry/'+self.csvFilename+'.csv', 'a')
            self.writer = csv.writer(self.csvfile)
            self.writer.writerow([str(self.time), "", str(self.ball_pos_x), ""])
            self.csvfile.close()
    
    def check_and_save(self):
        if self.counter % self.max_array_length == 0:
            print("writing telemetry data to csv.")
            # Pause
            rospy.wait_for_service('/gazebo/pause_physics')
            try:
                self.pause()
            except (rospy.ServiceException) as e:
                print ("/gazebo/pause_physics service call failed")

            with open('callback_update_times.csv', mode='w') as file:
                self.csvfile = open('runs/telemetry/'+self.csvFilename+'_callback_times.csv', 'a')
                self.writer = csv.writer(self.csvfile)
                for i in range(self.counter):
                    self.writer.writerow([self.wheel_update_times[i], self.ball_update_times[i], self.time_update_times[i]])
            self.counter = 0
            self.wheel_update_times = [0] * self.max_array_length
            self.ball_update_times = [0] * self.max_array_length
            self.time_update_times = [0] * self.max_array_length
            # Unpause simulation to make observations
            rospy.wait_for_service('/gazebo/unpause_physics')
            try:
                self.unpause()
            except (rospy.ServiceException) as e:
                print ("/gazebo/unpause_physics service call failed")

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
                print("ball missed")
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

        cv2.waitKey(1)
                
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        current_time = self.time
        dt = current_time - self.prev_time
        self.ball_pos_x = None

        while self.ball_pos_x is None:
            pass

        x_pos = self.ball_pos_x
        print("seconds waited to acquire ball pos: " + str(self.time-current_time))
        if dt == 0: # avoid division by zero
            x_speed = 0
        else:
            x_speed = (x_pos - self.prev_ball_pos) / dt

        # Update previous error and time for next iteration
        self.prev_ball_pos = x_pos
        self.prev_time = current_time

        # Take action        
        self.joint_pub.publish(action)

        # Define state  
        state = [x_pos, x_speed]

        # Check for end condition
        while self.ball_pos_y == None:
            pass
        done = bool(abs(self.ball_pos_y) < self.y_coord_reset_threshold)
                
        reward = 2-abs(x_pos)/self.x_coord_limit*2
        # print('ball pos: ' , x_pos)
        # print('reward ',reward)
        return state, reward, done, {}
    
    def reset(self): 
        print("Starting reset sequence")
        velocity_threshold = 0.01
        Kp = 0.05 
        Kd = 0.1
        prev_error = 0  

        while self.wheel_vel is None:
            pass
        i = 0
        while abs(self.wheel_vel) > velocity_threshold:
            start_time = self.time
            
            error = self.wheel_vel
            derivative = (error - prev_error)  
            effort = -Kp * error - Kd * derivative  
            self.joint_pub.publish(effort)
            
            prev_error = error  
            i+=1
            # if error * prev_error < 0 and error < 0.1:
            #     break
            while self.time < start_time + 0.01:
                pass
            self.wheel_vel = None
            while self.wheel_vel is None:
                pass
            # print("resetting wheel vel: " + str(error))

        self.joint_pub.publish(0.0)

        state_msg = ModelState()
        state_msg.model_name = 'ball'
        r = self.x_coord_limit
        x = np.random.uniform(-r/2,r/2)
        state_msg.pose.position.x = x
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.375 - (r - np.sqrt(r**2-x**2))
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0
        self.prev_ball_pos = x
        self.ball_pos_y = None
        rospy.wait_for_service('/gazebo/set_model_state')
        print('ball reset, loops taken: ' + str(i))
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
        except rospy.ServiceException:
            print( "Service call failed")
        return [x, 0] # TODO: Only approximately correct wheel velocity after reset
