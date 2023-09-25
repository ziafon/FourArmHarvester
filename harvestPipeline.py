#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import os,sys
import rospy
import numpy as np
from robot_msgs.msg import robotcore, hardware, messages
from movement import Movement
import _thread
from armSchedule import Schedule
from config import jointParam, jointReachThreshold, switchMap, gripperMap


'''
机械臂任务作业流程控制，将作业目标变换为个关节的期望位置，发送控制模块，同时接收控制模块返回的运行状态，开展调度。
每个机械臂设为一个作业单元unit，一堆耦合的机械臂设为一组，记为agent。
采用多线程，所有的线程均为阻塞操作。
机器人各臂伸抓缩放存在先后执行顺序，故占用一个线程。

z   操作可以和缩回、放置同时进行，但是一个智能体内的两个单元并不会同时执行对准动作，

所以为一个智能体的对准操作建立一个线程 ，总计六个线程。
一个线程中，每个动作都采用阻塞运行，完成后返回全局状态表示。
'''

class Movement():

        def __init__(self):
                
                rospy.init_node('harvest_control')

                self.lowerPub = rospy.Publisher('lower_feedback', messages, queue_size=1)
                self.upperPub = rospy.Publisher('upper_feedback', messages, queue_size=1)
                self.succeed = messages(jointParam)
                self.motors = Schedule()
                self.movement = Movement()


        def approach(self, msgs, joint):
                target = msgs.target
                agent = msgs.agent
                arm = msgs.arm
                v = target[0]
                h = target[1]
                j = joint[1] if arm == 'left' else joint[2]
                self.motors.desirePosition[joint[0]] = v
                self.motors.desirePosition[j] = h
                rospy.sleep(0.2)
                while not rospy.is_shutdown():
                        rospy.sleep(0.1)
                        deltaVertical = np.abs(v - self.motors.currentPosition[joint[0]])
                        deltaHorizontal = np.abs(h - self.motors.currentPosition[j])
                        if deltaVertical <  jointReachThreshold['vertical'] and deltaHorizontal < jointReachThreshold['horizontal']:
                                break
                self.succeed.state = 'approached'
                if agent == 'lower': self.lowerPub.publish(self.succeed)
                if agent == 'upper': self.upperPub.publish(self.succeed)


        def pick_place(self, msgs, joint):
                target = msgs.target
                agent = msgs.agent
                arm = msgs.arm
                e = target[2]
                # 气缸伸出
                self.movement.cylinderExtend(switchMap[agent+'_'+arm])
                self.motors.desirePosition[joint] = e
                rospy.sleep(2)
                while not rospy.is_shutdown():
                        rospy.sleep(0.1)
                        deltaExtend = np.abs(e - self.motors.currentPosition[joint[0]])
                        if deltaExtend <  jointReachThreshold['extend']:
                                break
                self.movement.grasp(gripperMap[agent+'_'+arm][4], gripperMap[agent+'_'+arm][0])
                rospy.sleep(1.5)
                self.movement.roll(gripperMap[agent+'_'+arm][4], gripperMap[agent+'_'+arm][2])
                rospy.sleep(2)
                self.succeed.state = 'picked'
                if agent == 'lower': self.lowerPub.publish(self.succeed)
                if agent == 'upper': self.upperPub.publish(self.succeed)
                # 气缸缩回
                self.movement.cylinderRetract(switchMap[agent+'_'+arm])
                self.motors.desirePosition[joint] = 0
                rospy.sleep(2)
                while not rospy.is_shutdown():
                        rospy.sleep(0.1)
                        deltaExtend = np.abs(0 - self.motors.currentPosition[joint[0]])
                        if deltaExtend <  jointReachThreshold['extend']:
                                break
                self.movement.grasp(gripperMap[agent+'_'+arm][4], gripperMap[agent+'_'+arm][1])
                rospy.sleep(1.5)
                self.movement.roll(gripperMap[agent+'_'+arm][4], gripperMap[agent+'_'+arm][3])
                rospy.sleep(2)
                self.succeed.state = 'placed'
                if agent == 'lower': self.lowerPub.publish(self.succeed)
                if agent == 'upper': self.upperPub.publish(self.succeed)
                






