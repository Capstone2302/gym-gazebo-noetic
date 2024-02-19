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
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
import time
from rosgraph_msgs.msg import Clock

class CommandToJointState:

    def __init__(self):
        self.center_pixel = 399
        self.Kp = 10
        self.Ki = 0
        self.Kd = 0.5
        self.prev_err = 0
        self.prev_time = 0
        self.time = 0
        self.wheel_write = 0.0
        self.wheel_vel_read = 0.0
        self.ball_pos_x = 0
        self.ball_pos_y = 0
        self.integral = 0
        self.prev_err=0
        self.curr_loop = 1
        self.reset_time = 5
        self.joint_name = 'rev'
        self.joint_state = JointState()
        self.joint_state.name.append(self.joint_name)
        self.joint_state.position.append(0.0)
        self.joint_state.velocity.append(0.0)
        self.bridge = CvBridge()
        self.joint_pub = rospy.Publisher("/wheel/rev_effort_controller/command", Float64, queue_size=1)
        self.ball_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.get_ball_pos_callback, queue_size=1)
        self.wheel_sub = rospy.Subscriber('/wheel/joint_states', JointState, self.get_wheel_pos_callback, queue_size=1)
        self.sim_time_sub = rospy.Subscriber("/clock", Clock, self.get_sim_time)
        # self.ball_sub = message_filters.Subscriber("/wheel/camera1/image_raw", Image)
        # ats = ApproximateTimeSynchronizer([self.wheel_sub, self.ball_sub], queue_size=5, slop=0.1)
        self.reset_ball_pos()
        # ats.registerCallback(self.my_callback)

    def get_sim_time(self, data):
        self.time = data.clock.secs + data.clock.nsecs/(1e9)
        if self.time > self.curr_loop * self.reset_time:
            self.reset_ball_pos()
            self.curr_loop+=1

    def PID_control(self):
        error = self.ball_pos_x

        # Calculate derivative of the error
        current_time = self.time
        dt = current_time - self.prev_time
        if dt == 0:
            # Avoid division by zero
            derivative = 0
        else:
            derivative = (error - self.prev_err) / dt

        # Update previous error and time for next iteration
        self.prev_err = error
        self.prev_time = current_time

        # Calculate control output with PID terms
        proportional = -self.Kp * error
        integral = self.Ki * self.integral + error * dt
        derivative = - self.Kd * derivative
        self.integral = integral  # Update integral term for next iteration

        self.wheel_write = proportional + integral + derivative

        # Publish control output
        self.joint_pub.publish(self.wheel_write)

    def reset_ball_pos(self):    
        print("starting reset sequence")
        velocity_threshold = 0.01
        Kp = 0.1  # Proportional gain
        while abs(self.wheel_vel_read) > velocity_threshold:
            effort = -Kp * self.wheel_vel_read
            print("Wheel velocity feedback (during reset loop): " + str(self.wheel_vel_read))
            self.joint_pub.publish(effort)
            print("New wheel velocity published")

        self.joint_pub.publish(0.0)

        state_msg = ModelState()
        state_msg.model_name = 'ball'
        r = 0.1524
        x = np.random.uniform(-r/2,r/2)
        state_msg.pose.position.x = x
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.375 - (r - np.sqrt(r**2-x**2))
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

    def get_wheel_pos_callback(self, msg):
        self.wheel_vel_read = msg.velocity[0]
        
        # print('wheel vel read: '+ str(self.wheel_vel_read ))
        # self.wheel_write = self.wheel_vel_read +1
        # self.joint_pub_velocity.publish(self.wheel_write)
        # print('position published: '+ str(self.wheel_vel_read +1))

    def get_ball_pos_callback(self, msg):
        self.ball_pos_x = msg.pose[1].position.x
        self.ball_pos_y = msg.pose[1].position.z
        print("Coords pos: " + str( msg.pose[1].position))
        if self.ball_pos_y < 0.1:
            self.reset_ball_pos()
                
        self.PID_control()

        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('command_to_joint_state')
    command_to_joint_state = CommandToJointState()
    rospy.spin()
