#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_srvs.srv import Empty
from datetime import datetime
import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest
  
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.msg import ModelStates
import time
from rosgraph_msgs.msg import Clock

class CommandToJointState:

    def __init__(self):
        self.center_pixel = 399
        self.Kp = np.pi*3
        self.Ki = 0
        self.Kd = 0
        self.prev_err = 0
        self.dt = 1
        self.time = 0
        self.prev_time = 0
        self.wheel_PID_output = 0.0
        self.wheel_vel = 0.0
        self.ball_pos_x = 0
        self.ball_pos_y = 0
        self.integral = 0
        self.prev_err=0
        self.joint_name = 'rev'
        self.wheel_pos_publish = 0
        self.wheel_pos = 0
        self.joint_state = JointState()
        self.joint_state.name.append(self.joint_name)
        self.joint_state.position.append(0.0)
        self.joint_state.velocity.append(0.0)
        self.bridge = CvBridge()
        self.joint_pub = rospy.Publisher("/wheel/rev_position_controller/command", Float64, queue_size=1)
        # self.wheel_sub = message_filters.Subscriber('/wheel/joint_states', JointState)
        self.wheel_sub = rospy.Subscriber('/wheel/joint_states', JointState, self.get_wheel_callback, queue_size=1)
        # self.ball_sub = message_filters.Subscriber("/wheel/camera1/image_raw", Image)
        self.ball_sub_cam = rospy.Subscriber("/wheel/camera1/image_raw", Image, self.camera_callback, queue_size=1)
        self.ball_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.get_ball_pos_callback, queue_size=1)
        # ats = ApproximateTimeSynchronizer([self.wheel_sub, self.ball_sub], queue_size=5, slop=0.1)
        self.resetting = False
        # ats.registerCallback(self.my_callback)weel vel published: 0.0
        self.sim_time_sub = rospy.Subscriber("/clock", Clock, self.get_sim_time)
        # self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_ball_pos()
        self.controller()
        # self.update_PID_constants()

    def get_sim_time(self, data):
        self.time = data.clock.secs + data.clock.nsecs/(1e9)

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
                # self.ball_pos_y_camera = y
                # self.ball_pos_x = x - self.center_pixel
                cv2.circle(output, (x, y), r, (255, 0, 0), 4)               
                # if self.ball_pos_y > 450:
                #     self.reset_ball_pos()
                
                # self.PID_control()
            if len(circles) == 0:
                print("ball missed")
        # print("Camera ball position: " + str(self.ball_pos_x))
                
        # cv2.imshow("output", np.hstack([cv_image, output]))
        # cv2.imshow("Image window", output)


        # cv2.waitKey(1)
    def camera_callback(self, img_msg):
        self.raw_image = img_msg

        # if not self.resetting:
        #     self.process_img(self.raw_image)
        #     # print("ball_pos_x: ",self.ball_pos_x)
 
    def get_ball_pos_callback(self, msg):
        self.ball_pos_x = msg.pose[1].position.x
        self.ball_pos_y = msg.pose[1].position.z
        # print('ball state read')

        
    def controller(self):
        while True:
            # print("controller running")
            if not self.resetting:
                self.ball_pos_x = None
                self.wheel_pos = None
                while (self.ball_pos_x is None) or (self.wheel_pos is None):
                    a = 1
                
                print(self.wheel_pos)
                if (self.ball_pos_y) < 0.06:
                    # print("*RESET*")
                    self.reset_ball_pos()
                else:
                    r = np.sqrt(self.ball_pos_x**2 + (self.ball_pos_y-0.2794)**2)

                    if self.wheel_pos > 2*np.pi/2 and self.wheel_pos < 6:
                        self.joint_pub.publish(float(self.wheel_pos-np.pi/4))
                        print('catch ball')
                    elif r > 0.145:
                        self.PID_mode()
                        print("PID MODE")
                    else:
                        print("tilt mode")
                        self.joint_pub.publish(float(self.wheel_pos+0.1))
        
    def PID_mode(self):
        error = self.ball_pos_x

        # Calculate derivative of the error
        current_time = self.time
        dt = current_time - self.prev_time
        # print("dt = ", dt)
        if dt == 0:
            # Avoid division by zero
            derivative = 0
            # print
            # ('*** dt = ',str(dt))
        else:
            derivative = (error - self.prev_err) / dt

            # Update previous error and time for next iteration
            self.prev_err = error
            self.prev_time = current_time

            # Calculate control output with PID terms
            proportional = self.Kp * error
            integral = self.Ki * (self.integral + error * dt)
            # print('integral: ', str(integral))
            derivative = self.Kd * derivative
            self.integral = integral  # Update integral term for next iteration

            self.wheel_PID_output = -(proportional + integral + derivative)
            print("prop: ", proportional)
            print("deriv: ", derivative)
            # Publish control output

            self.joint_pub.publish(float(self.wheel_PID_output + self.wheel_pos))
            # print('wheel pos published: '+ str(self.wheel_PID_output))
        
    def update_PID_constants(self):
        # Define some variables
        while True:
            # Get user input
            user_input = input("Enter var name: ")

            # Check user input and modify variables accordingly
            if user_input == 'ki':
                new_value = float(input("Enter the new value for ki: "))
                self.Ki = new_value
            elif user_input == 'kp':
                new_value = float(input("Enter the new value for kp: "))
                self.Kp = new_value
            elif user_input == 'kd':
                new_value = float(input("Enter the new value for kd: "))
                self.Kd = new_value
            elif user_input == 'pos':
                new_value = float(input("Enter the new value for pos: "))
                self.joint_pub.publish(float(new_value))
            else:
                print("Invalid input")

            # Print updated variables
            print("Updated kp:", self.Kp)
            print("Updated ki:", self.Ki)
            print("Updated kd:", self.Kd)      

    def get_wheel_callback(self, msg):
        self.wheel_pos = msg.position[0]%(2*np.pi)
        self.wheel_vel = msg.velocity[0]
        self.wheel_eff = msg.effort[0]

        # print('wheel state read')
        # print("eff: ",self.wheel_eff)
        # self.publish_input_position()
       
    def reset_wheel(self):    
        self.joint_pub.publish(float(0)) # zero effort
        msg = SetModelConfigurationRequest()
        msg.model_name = 'wheel'
        msg.joint_names = ['rev']
        msg.joint_positions = [float(0)]

        rospy.wait_for_service('/gazebo/set_model_configuration')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
            resp = set_state( msg )
        except rospy.ServiceException:
            print( "Service call failed")

        time.sleep(1)

    def reset_ball_pos(self):    
        self.ball_pos_x = None
        self.wheel_pos = None
        self.resetting = True
        self.reset_wheel()
            
        state_msg = ModelState()
        state_msg.model_name = 'ball'
        x=0
        state_msg.pose.position.x = x
        state_msg.pose.position.y = 0
        # state_msg.pose.position.z = 0.53
        state_msg.pose.position.z = 0.41
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

        self.integral = 0  

        time.sleep(1)
        self.resetting = False
        

    # def update_loop(self):
    #     while True:
    #         # # Get user input
    #         # user_input = (input("Enter Position"))
    #         # if user_input:
    #         #     self.user_input_wheel_pos = float(user_input)
    #         # Unpause simulation to make observations
    #         rospy.wait_for_service('/gazebo/unpause_physics')
    #         try:
    #             self.unpause()
    #         except (rospy.ServiceException) as e:
    #             print ("/gazebo/unpause_physics service call failed")

    #         time.sleep(0.01)

    #         # Pause
    #         rospy.wait_for_service('/gazebo/pause_physics')
    #         try:
    #             self.pause()
    #         except (rospy.ServiceException) as e:
    #             print ("/gazebo/pause_physics service call failed")
    #         input("paused")
            

    # def publish_input_position(self):
    #     # Define some variables
    #     msg = SetModelConfigurationRequest()
    #     msg.model_name = 'wheel'
    #     msg.joint_names = ['rev']
    #     # Get user input

    #     msg.joint_positions = [float(self.user_iwaitingnput_wheel_pos)]
    #     rospy.wait_for_service('/gazebo/set_model_configuration')
    #     try:
    #         set_state = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
    #         resp = set_state( msg )
    #     except rospy.ServiceException:
    #         print( "Service call failed")



if __name__ == '__main__':
    rospy.init_node('command_to_joint_state')
    command_to_joint_state = CommandToJointState()
    rospy.spin()
