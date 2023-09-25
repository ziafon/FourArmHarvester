#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from robot_msgs.msg import messages
from config import jointMap
import _thread
from task import simple_plan
from config import settedTargets

class Plan():
    def __init__(self):

        self.mainPub = rospy.Publisher('robot_command', messages, queue_size=3)
        self.Targets = {'lower':[], 'upper': []}
        self.pointer = {'lower':0, 'upper': 0}


    '''
    plan.current 和plan.next 分别取各个agent的当前坐标和下一个坐标
    plan.pointer 记录了每个agent任务列表的指针位置
    plan.Targets 记录了所有agent需要到达的坐标的列表

    '''


    def update(self, agent):
        self.pointer[agent] += 1
        if self.pointer[agent] < len(self.Targets[agent]):
            return True
        else:
            return False

    def reset(self, agent):
        self.pointer[agent] = 0
        self.Targets[agent] = []


    '''
    plan.state 表示当前计划的状态，有三种状态：Done,Planning,Error
    '''

    def state(self, agent):
        if self.pointer[agent] == len(self.Targets[agent]):
            return 'Done'
        else:
            return 'Planning'
    def count(self, agent):
        num_picked = self.pointer[agent] 
        num_nonPicked = len(self.Targets[agent]) -  num_picked
        return num_picked, num_nonPicked
    def Error(self, agent):
        if self.pointer[agent] > len(self.Targets[agent]):
            return 'Pointer is larger than list length'
        elif self.pointer[agent] <= 0:
            return 'Invalid Pointer'



    def getTaskList(self, distribution):
        flag = {'lower': True, 'upper': True}
        self.Targets = simple_plan(np.array(distribution).reshape(-1, 4), self.Targets)
        for agent in ['lower', 'upper']:
            flag[agent] = False if len(self.Targets[agent]) == 0 else True
        return flag
    
    def useSettedList(self):
        self.Targets = settedTargets


    def targetPublish(self, agent):
        target = self.Targets[agent][self.pointer[agent]]
        currentTask = messages()
        currentTask.action = 'picking'
        currentTask.target = target[0:3]
        currentTask.agent = agent
        currentTask.arm = 'left' if target[4] == 0 else 'right'
        self.mainPub.publish(currentTask)
        print('目标发送')





