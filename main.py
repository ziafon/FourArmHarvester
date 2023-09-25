#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os,sys
import rospy
import numpy as np
from robot_msgs.msg import messages
from plan import Plan
import subprocess


'''
顶层控制，负责所有功能模块的启动和配置，同时管理整体作业流程。
'''

# 启动感知功能节点
sub1 = subprocess.Popen(['python3', 'perception.py'])
# 启动调度功能节点
sub2 = subprocess.Popen(['python3', 'schedule.py'])
# 启动相机图像接收节点
sub3 = subprocess.Popen(['python3', 'multiCam.py'])
# 启动硬件驱动
sub4 = subprocess.Popen(['python3', 'drive.py'])


class Harvester():

    def __init__(self):
        rospy.init_node('robot_harvester')
        self.plan = Plan()
        self.targetExist = {'lower': False, 'upper': False} 
        self.action = messages()
        self.distribution = None
        self.harvestStop = False
        self.pub =  rospy.Publisher('robot_command', messages, queue_size=5)
        rospy.Subscriber('robot_command', messages, callback=self.mainCallback, queue_size=5)
        rospy.spin()


    def mainCallback(self, msgs):
        if msgs.command == 'observation':
            self.action.action = 'observation'
            self.pub.publish(self.action)
        if msgs.command == 'origin':
            self.harvestStop = True
            self.action.action = 'origin'
            self.pub.publish(self.action)
        if msgs.command == 'useFixed':
            for agent in ['lower', 'upper']:
                self.plan.reset(agent)
                self.flag[agent] = True
            self.plan.useSettedList()
            print(self.plan.Targets)
        if msgs.command == 'useDection':
            self.distribution = msgs.objects
            if len(self.distribution) > 0:
                for agent in ['lower', 'upper']:
                    self.plan.reset(agent)
                self.flag = self.plan.getTaskList(self.distribution)
                print(self.plan.Targets)
            else:
                print('没有目标，请重新检测。')
        if msgs.command == 'excution': 
            for agent in ['lower', 'upper']:
                if self.targetExist[agent] == True:
                    self.plan.targetPublish(agent)
        if msgs.workingState == 'picked':
            agent = msgs.agent
            flag = self.plan.update(agent)
            if flag == True:
                self.plan.targetPublish(agent)


if __name__ == '__main__':
    try:
        Harvester()
    except rospy.ROSInterruptException:
        pass







