#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
import rospy
from robot_msgs.msg import messages, hardware
from motorControl import Control
from config import jointParam, reachedThreshold, switchMap, gripperMap
from utils import single_adaptive

class Movement():

    def __init__(self):

        self.msgPub = rospy.Publisher('message', messages, queue_size=3)
        self.ctlPub = rospy.Publisher('robot_control', hardware, queue_size=3)

        self.messages = messages()
        self.move = hardware()
        self.move.target_velocity = [0 for _ in range(10)]
        self.ioMap = {'lower_left':3, 'lower_right':1, 'upper_left':0, 'upper_right':2}
        self.armID = ['DL', 'DR', 'UL', 'UR']
        self._pass = {'lower_left': False, 'lower_right': False, 'upper_left': False, 'upper_right': False}
        self.gripperPub = rospy.Publisher('gripper', hardware, queue_size=3)
        self.gripperOpt = hardware()
        self.gripperOpt.gripper_operation = [1600,    800,   3300,  2700,  200,   1600,   50,    1800]
        self.gripperPub.publish(self.gripperOpt)
        self.gripper = {'lower_left': [200, 1650], 'lower_right': [1600, 300], 'upper_left': [50, 1350], 'upper_right': [1800, 400]}
        self.IDX = {'lower_left': 0, 'lower_right': 1, 'upper_left': 2, 'upper_right': 3}
        self.lowerPub = rospy.Publisher('lower_feedback', messages, queue_size=1)
        self.upperPub = rospy.Publisher('upper_feedback', messages, queue_size=1)
        self.motors = Control()

    def origin(self):
        # self.messages.action = 'origining'
        # self.msgPub.publish(self.messages)
        # print('回原点')
        rospy.sleep(0.2)
        # 发布控制信息
        self.ctlPub.publish(self.move)
        # 设定期望位置
        ArmMotorDesiredPosition = np.array([-0, 0, -0, 0, 0, -0, 0, 0, 0, 0])
        # 获取当前位置
        ArmMotorCurrentPosition = self.motors.currentPosition
        # 初始化计数器
        count = np.zeros(10)
        # 当程序没有被关闭时
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            # 更新当前位置
            ArmMotorCurrentPosition = self.motors.currentPosition
            # 计算当前位置与期望位置的差值
            delta = ArmMotorCurrentPosition - ArmMotorDesiredPosition
            # 根据差值调整速度
            for i in range(10):
                if np.abs(delta[i]) > 0.005:
                    self.move.target_velocity[i] = int(single_adaptive(delta[i], i))
                else:
                    count[i] = 1
                    self.move.target_velocity[i] = 0
            # 发布控制信息
            self.ctlPub.publish(self.move)
            # 若所有轴都满足条件，退出循环
            if np.sum(count) == 10:
                self.move.target_velocity = [0 for i in range(10)]
                self.move.switch_open = []
                self.move.switch_close = [0, 1, 2, 3]
                self.ctlPub.publish(self.move)
                break
        rospy.sleep(0.5)
        self.motors.desirePosition = ArmMotorCurrentPosition
        # self.msgPub.publish(self.messages)


    def observation(self):
        # 将四个机械臂移动至固定构型，来获取全局果实分布
        # self.messages.action = 'observing'
        # self.msgPub.publish(self.messages)
        # print('去观测位')
        rospy.sleep(0.2)
        ArmMotorDesiredPosition = np.array([-0.3, 0.1, -0.54, 0.54, 0.54, -0.54, 0, 0, 0, 0])
        ArmMotorCurrentPosition = self.motors.currentPosition
        # 若到达指定位置，isReached 元素赋为1
        count = np.zeros(10)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            ArmMotorCurrentPosition = self.motors.currentPosition
            delta = ArmMotorCurrentPosition - ArmMotorDesiredPosition
            for i in range(10):
                if np.abs(delta[i]) > 0.005:
                    self.move.target_velocity[i] = int(single_adaptive(delta[i], i))
                else:
                    count[i] = 1
                    self.move.target_velocity[i] = 0
            self.ctlPub.publish(self.move)
            # 满足条件，退出
            if np.sum(count) == 10:
                self.move.target_velocity = [0 for i in range(10)]
                self.move.switch_open = []
                self.move.switch_close = [0, 1, 2, 3]
                self.ctlPub.publish(self.move)
                break
        rospy.sleep(0.5)
        self.motors.desirePosition = ArmMotorCurrentPosition
        # self.messages.action = 'observed'
        # self.msgPub.publish(self.messages)
        # print('到达观测位')


    def approach(self, target, jointV, jointH):
        v = target[0]
        h = target[1]
        self.motors.desirePosition[jointV] = v
        self.motors.desirePosition[jointH] = h
        rospy.sleep(0.2)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            deltaVertical = np.abs(v - self.motors.currentPosition[jointV])
            deltaHorizontal = np.abs(h - self.motors.currentPosition[jointH])
            if deltaVertical <  reachedThreshold['vertical'] and deltaHorizontal < reachedThreshold['horizontal']: break


    def pick(self, unit, joint, target):
        e = target[2]
        # 气缸伸出
        self.cylinderExtend(switchMap[unit])
        self.motors.desirePosition[joint] = e
        rospy.sleep(2)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            deltaExtend = np.abs(e - self.motors.currentPosition[joint])
            if deltaExtend < reachedThreshold['extend']:
                break
        # self.grasp(gripperMap[unit][4], gripperMap[unit][0])
        rospy.sleep(1.5)
        # self.roll(gripperMap[unit][4], gripperMap[unit][2])
        rospy.sleep(2)


    def place(self, unit, joint):
        # 气缸缩回
        self.cylinderRetract(switchMap[unit])
        self.motors.desirePosition[joint] = 0
        rospy.sleep(2)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            deltaExtend = np.abs(0 - self.motors.currentPosition[joint])
            if deltaExtend < reachedThreshold['extend']:
                break
        # self.grasp(gripperMap[unit][4], gripperMap[unit][1])
        rospy.sleep(1.5)
        # self.roll(gripperMap[unit][4], gripperMap[unit][3])
        rospy.sleep(2)


    def extend(self, unit, joint, position):
        # 气缸伸出
        self.cylinderOperation(self, self.ioMap[unit], 1)
        # 滑动模组伸出
        self.joint_position_publish(self, joint, position)
        count = 0
        thre = 12
        while count < thre:
            rospy.sleep(0.2)
            count += 1
            t = 1
            # 紧急缩回触发
            if self._pass[unit] == True:
                # 气缸和模组缩回
                self.cylinderOperation(self, self.ioMap[unit], 0)
                self.joint_position_publish(self, joint, 0)
                rospy.sleep(0.25)
                self._pass[unit] = False
                again = False
                for _ in range(9):
                    # 再次伸出，手爪变化角度
                    rospy.sleep(0.2)
                    if self._pass[unit] == True:
                        self.roll(unit, t)
                        self.cylinderOperation(self, self.ioMap[unit], 1)
                        self.joint_position_publish(self, joint, position)
                        thre += 10
                        again = True
                        rospy.sleep(0.25)
                        self._pass[unit] = False
                        t += 1
                        break
                if again == False:
                    return 'grasped'
        # 等待执行完毕
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if self.isReached[joint] == True:
                break
        return 'extended'




    def retract(self, unit, joint, position):
        self.cylinderOperation(self, self.ioMap[unit], 0)
        self.joint_position_publish(self, joint, position)
        rospy.sleep(2)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if self.isReached[joint] == True:
                break
        return 'retracted'


    def cylinderExtend(self, id):
        self.move.switch_open =[]
        self.move.switch_close = [id]
        self.ctlPub.publish(self.move)


    def cylinderRetract(self, id):
        self.move.switch_open =[]
        self.move.switch_close = [id]  
        self.ctlPub.publish(self.move)


    def roll(self, unit, t):
        val = self.gripper[unit][0] + 100 * t
        self.gripperOpt.gripper_operation[self.IDX[unit] + 4] = val
        self.gripperPub.publish(self.gripperOpt)

    def back(self, unit):
        val = self.gripper[unit][1]
        self.gripperOpt.gripper_operation[self.IDX[unit] + 4] = val
        self.gripperPub.publish(self.gripperOpt)  
