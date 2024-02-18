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
from gazebo_msgs.srv import SetModelState
from rosgraph_msgs.msg import Clock
import pandas as pd
import time

class CommandToJointState:

    def __init__(self):
        self.joint_name = 'rev'
        self.joint_state = JointState()
        self.joint_state.name.append(self.joint_name)
        self.joint_state.effort.append(0.0)
        self.joint_state.effort.append(0.0)
        self.bridge = CvBridge()
        self.joint_pub = rospy.Publisher("/wheel/rev_position_controller/command", Float64, queue_size=1)
        self.wheel_sub = rospy.Subscriber('/wheel/joint_states', JointState, self.get_wheel_pos_callback,queue_size=1)
        self.sim_time_sub = rospy.Subscriber("/clock", Clock, self.get_sim_time)
        self.curr_loop = 1
        self.reset_time = 0.5

        self.setpoint_effort = -1
        self.time_effort_set = 0
        self.time_effort_received = None
        self.data_points = []
        self.buffer_size = 100
        self.data_df = pd.DataFrame(columns=['Time', 'Setpoint effort', 'Actual effort'])

    def get_sim_time(self, data):
        self.time = data.clock.secs + data.clock.nsecs/(1e9)
        self.effort_setpoint_setter()
    
    def effort_setpoint_setter(self):
        if self.time > self.curr_loop * self.reset_time:
            if self.curr_loop % 2 == 0:
                self.setpoint_effort = -1
            else:
                self.setpoint_effort = 1
            self.curr_loop+=1
            self.setpoint_effort = 0
        self.setpoint_effort = 1/self.reset_time*(self.time - (self.curr_loop-1) * self.reset_time)
        if self.curr_loop % 2 == 0:
            self.setpoint_effort *= -1
        self.set_effort_command(self.setpoint_effort)

    def set_effort_command(self, effort):
        self.time_effort_set = self.time
        self.joint_pub.publish(effort)

    def get_wheel_pos_callback(self, msg):
        self.actual_effort = msg.position[0]
        
        self.data_points.append({'Time': self.time,
                                         'Setpoint effort': self.setpoint_effort,
                                         'Actual effort': self.actual_effort})

        if len(self.data_points) >= self.buffer_size:
            self.append_buffer_to_dataframe()

    def append_buffer_to_dataframe(self):
        buffer_df = pd.DataFrame(self.data_points)

        self.data_df = pd.concat([self.data_df, buffer_df], ignore_index=True)

        self.data_points = []

    def shutdown(self):
        print("Shutdown function triggered")
        self.data_df.to_csv('~/Documents/Capstone/gym-gazebo-noetic/runs/telemetry/motor_test_control_response_' + datetime.now().strftime("%b%d-%H-%M-%S-rlwheel") + ".csv", index=False)

if __name__ == '__main__':
    rospy.init_node('command_to_joint_state')
    command_to_joint_state = CommandToJointState()

    rospy.on_shutdown(command_to_joint_state.shutdown)
    
    rospy.spin()
