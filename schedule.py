#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from robot_msgs.msg import messages
import _thread
from movement import Movement
from config import jointMap
from utils import invCalculate, tf_base2fake_listener
from config import jointMap, armName, stretchDIR

'''
调度功能的总入口，负责机器人所有行为的执行。
通过ROS话题指令执行相应的行为，包括回原点，去观测位，以及机械臂采摘行为等。
其中为每个目标采摘流程开辟一个线程，目标采摘结束后线程自动销毁。

'''

class Schedule():

    def __init__(self):
         
        rospy.init_node('robot_harvest')
        self.movement = Movement()
        rospy.sleep(0.5)
        self.tf_linsener = tf_base2fake_listener(armName)
        self.canExtend = {'lower_left':True, 'lower_right':True, 'upper_left':True, 'upper_right':True}
        self.pub = rospy.Publisher('robot_command', messages, queue_size=5)
        rospy.Subscriber('robot_command', messages, callback=self.Callback, queue_size=3)
        rospy.spin()


    def Callback(self, msgs):
        if msgs.action == 'observation':
            print('去观测位')
            self.movement.motors.run = False
            self.movement.observation()
            self.movement.motors.run = True
            print('到达观测位')
        if msgs.action == 'origin':
            print('回原点')
            self.movement.motors.run = False
            self.movement.origin()
            print('到达原点')
        if msgs.action == 'picking':
            agent = msgs.agent
            arm = msgs.arm
            target = msgs.target
            # 手臂编号确定
            i = 0 if agent == 'lower' else 1
            j = 0 if arm == 'left' else 1
            joint = jointMap[agent]
            # 坐标转换，生成关节位移
            tf_listener = self.tf_linsener[2*i+j]
            id = armName[2*i+j]
            direction = stretchDIR[agent + '_' + arm]
            joint_desire_position = invCalculate(target, tf_listener, id, direction)
            _thread.start_new_thread(self.harvestPipeline, (agent, arm, joint, joint_desire_position))


    def harvestPipeline(self, agent, arm, joint, position):
        print('开始采摘：', position)
        unit = agent + '_' + arm
        jointHorizontal = joint[1] if arm == 'left' else joint[2]
        jointExtend = joint[3] if arm == 'left' else joint[4]
        avoid = 1 if agent == 'lower' else 2
        jointAviod = 9 - jointHorizontal if arm == 'left' else 5 - jointHorizontal
        self.movement.motors.priority[avoid] = jointHorizontal
        self.movement.motors.avoid[avoid] = jointAviod
        self.movement.approach(position, joint[0], jointHorizontal)
        self.movement.motors.priority[0] = joint[0]
        self.movement.motors.avoid[0] = 1- joint[0]
        while self.canExtend[unit] == False:
            rospy.sleep(0.1)
        self.movement.pick(unit, jointExtend, position)
        self.canExtend[unit] = False
        call4next = messages()
        call4next.workingState = 'picked'
        self.movement.motors.priority[0] = 1 - joint[0]
        self.movement.motors.avoid[0] = joint[0]
        call4next.agent = agent
        self.pub.publish(call4next)
        self.movement.place(unit, jointExtend)
        self.canExtend[unit] = True


if __name__=='__main__': 
    try:
        Schedule()
    except KeyboardInterrupt:
        rospy.loginfo('Hasta la Vista...')