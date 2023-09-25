#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import _thread
from robot_msgs.msg import hardware
from sensor_msgs.msg import JointState
from utils import distanceCalculate, adaptiveControl, unitConvertion
from config import _JointName, jointParam


class Control():

    def __init__(self):
        
        self.jointNum = jointParam['jointNum']

        self.jsPosition = [0.0 for _ in range(jointParam['urdfJointNum'])]
        self.currentPosition = np.zeros(self.jointNum)
        self.desirePosition = np.zeros(self.jointNum)
        self.control_publisher = rospy.Publisher('robot_control', hardware, queue_size=10)

        _thread.start_new_thread(self.motorFeedback,())
        _thread.start_new_thread(self.motorRun,())
        self.priority = [1, 5, 2]
        self.avoid = [0, 4, 3]
        self.run = False
        self.move = hardware()
        self.move.target_velocity = [0 for i in range(self.jointNum)]
        self.move.switch_close = [0, 1, 2, 3]
        self.mode = 'WAIT'


    def motorRun(self):
        # 定义发布关节消息
        js_pub = rospy.Publisher('joint_states', JointState, queue_size=5)
        js = JointState() # 定义机器人关节消息
        js.header.stamp = rospy.Time.now()
        js.name = _JointName
        SAFETHRESHOLD = [0.53, 1.15, 1.15] # 安全阈值，自定义硬编码
        while not rospy.is_shutdown():
            # print('here')
            rospy.sleep(0.1)
            # print(self.jsPosition)
            js.position = self.jsPosition
            js_pub.publish(js)
            if self.run == True:
                # 计算关键距离，共三个元素。0：上下导轨之间的距离， 1：左下和右下两臂横向距离，2：左上和右上两臂横向距离
                keyDistance = distanceCalculate(self.jsPosition)
                # 通过当前位置和期望位置生成自适应速度控制量
                self.targetVelocity = adaptiveControl(self.currentPosition - self.desirePosition) 
                for i in range(3):
                    if keyDistance[i] - SAFETHRESHOLD[i] > -0.05:
                        self.targetVelocity[self.avoid[i]] = 0
                    if keyDistance[i] - SAFETHRESHOLD[i] > 0:
                        self.targetVelocity[self.avoid[i]] = self.targetVelocity[self.priority[i]]
                self.move.target_velocity = np.int32(self.targetVelocity).tolist()
                self.control_publisher.publish(self.move)


    def motorFeedback(self):
        rospy.Subscriber('robot_feedback', hardware, callback=self.robot_callback ,queue_size=3)
        rospy.spin()
    
    def robot_callback(self, msgs):
        self.gripper_position = msgs.gripper_status[4:8]
        self.gripper_current = msgs.gripper_status[8:12]
        self.motor_position = msgs.actual_position
        self.motor_status = msgs.motor_status
        # print(self.motor_position)
        self.jsPosition, self.currentPosition = unitConvertion(self.jsPosition, self.motor_position)
        # 计算关键距离，共三个元素。0：上下导轨之间的距离， 1：左下和右下两臂横向距离，2：左上和右上两臂横向距离
        # self.important_distance = distanceCalculate(self.joint_positon)  