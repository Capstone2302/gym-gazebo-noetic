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
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from rosgraph_msgs.msg import Clock
import pandas as pd

import time

class CommandToJointState:

    def __init__(self):
        self.center_pixel = 399
        self.Kp = 1
        self.Ki = 0.0001
        self.Kd = 0
        self.prev_err = 0
        self.dt = (datetime.now() - datetime.now()).microseconds
        self.t = datetime.now()
        self.wheel_write = 0.0
        self.wheel_vel_read = 0.0
        self.ball_pos_x = 0
        self.ball_pos_y = 0
        self.integral = 0
        self.prev_err=0
        self.joint_name = 'rev'
        self.joint_state = JointState()
        self.joint_state.name.append(self.joint_name)
        self.joint_state.position.append(0.0)
        self.joint_state.velocity.append(0.0)
        self.bridge = CvBridge()
        self.joint_pub = rospy.Publisher("/wheel/rev_position_controller/command", Float64, queue_size=1)
        # self.wheel_sub = message_filters.Subscriber('/wheel/joint_states', JointState)
        self.wheel_sub = rospy.Subscriber('/wheel/joint_states', JointState, self.get_wheel_pos_callback, queue_size=1)
        # self.ball_sub = message_filters.Subscriber("/wheel/camera1/image_raw", Image)
        self.ball_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.get_ball_pos_callback, queue_size=1)
        # ats = ApproximateTimeSynchronizer([self.wheel_sub, self.ball_sub], queue_size=5, slop=0.1)
        self.reset_ball_pos()
        self.sim_time_sub = rospy.Subscriber("/clock", Clock, self.get_sim_time)

        self.data_points = []
        self.buffer_size = 100
        self.data_df = pd.DataFrame(columns=['Time', 'Ball position', 'Control Signal'])

    def get_sim_time(self, data):
        self.time = data.clock.secs + data.clock.nsecs/(1e9)

    def append_buffer_to_dataframe(self):
        buffer_df = pd.DataFrame(self.data_points)

        self.data_df = pd.concat([self.data_df, buffer_df], ignore_index=True)

        self.data_points = []

    def shutdown(self):
        print("Shutdown function triggered")
        self.data_df.to_csv('~/Documents/Capstone/gym-gazebo-noetic/runs/telemetry/pid_control_response' + datetime.now().strftime("%b%d-%H-%M-%S-rlwheel") + ".csv", index=False)


    def PID_control(self):
        # self.dt = ((datetime.now() - self.t).microseconds)/1000.0
        # self.t = datetime.now()
        # error = self.ball_pos_x
        # self.integral = self.integral + error*self.dt
        # derivative = (error - self.prev_err)/self.dt
        # self.wheel_write = self.wheel_vel_read + self.Kp*error + self.Ki*self.integral + self.Kd*derivative
        # self.prev_err = error
        if self.ball_pos_x < 0:
            self.wheel_write += abs(self.ball_pos_x)*self.Kp
        elif self.ball_pos_x > 0:
            self.wheel_write -= abs(self.ball_pos_x)*self.Kp
        else:
            self.wheel_write = 0

        self.joint_pub.publish(self.wheel_write)
        print('weel vel published: '+ str(self.wheel_write))
        self.data_points.append({'Time': self.time,
                                         'Ball position': self.ball_pos_x,
                                         'Control Signal': self.wheel_write})

        if len(self.data_points) >= self.buffer_size:
            self.append_buffer_to_dataframe()

    def reset_ball_pos(self):    
        # self.joint_pub.publish(float(0))
        time.sleep(0.5)
        state_msg = ModelState()
        state_msg.model_name = 'ball'
        self.wheel_write = 0
        r = 0.1524
        x = np.random.uniform(-r/4,r/4)
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
        msg_str = str(msg)
        i = msg_str.find('velocity: [')+11
        msg_str = msg_str[i:]
        i = msg_str.find(']')
        self.wheel_vel_read = float(msg_str[0:i])
        
        # self.wheel_write = self.wheel_vel_read +1
        # self.joint_pub.publish(self.wheel_write)
        # print('position published: '+ str(self.wheel_vel_read +1))

    def get_ball_pos_callback(self, msg):

        self.ball_pos_x = msg.pose[1].position.x
        self.ball_pos_y = msg.pose[1].position.z
        print("ball pos x read: " + str(self.ball_pos_x))
        self.PID_control()
        if self.ball_pos_y < 0.1:
            self.reset_ball_pos() 

if __name__ == '__main__':
    rospy.init_node('command_to_joint_state')
    command_to_joint_state = CommandToJointState()
    rospy.on_shutdown(command_to_joint_state.shutdown)
    rospy.spin()
