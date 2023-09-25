#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

_JointName = ['vertical_motion_lower_arm_joint',
                    'horizontal_motion_lower_left_arm_joint',
                    'stretch1_motion_lower_left_arm_joint',
                    'stretch2_motion_lower_left_arm_joint',
                    'horizontal_motion_lower_right_arm_joint',
                    'stretch1_motion_lower_right_arm_joint',
                    'stretch2_motion_lower_right_arm_joint',
                    'vertical_motion_upper_arm_joint',
                    'horizontal_motion_upper_left_arm_joint',
                    'stretch1_motion_upper_left_arm_joint',
                    'stretch2_motion_upper_left_arm_joint',
                    'horizontal_motion_upper_right_arm_joint',
                    'stretch1_motion_upper_right_arm_joint',
                    'stretch2_motion_upper_right_arm_joint']

jointParam = {'jointNum': 10, 'urdfJointNum': 14}

motorNum = 10

motorParam = {'vertical': 10000*40/0.18, 'horizontal': 10000*10/0.16, 'extend': 10000*10/0.16}

gripper_operation = [1600,    800,   3300,  2700,  200,   1600,   50,    1800]
gripper_operation = [1600,    800,   3300,  2700,  200,   1600,   50,    1800]

arm_thread_config = {'lowerGroup': [1, 5, 4],
         'upperGroup': [0, 2, 3]
         }

jointMap = {'lower':[1, 5, 4, 9, 8], 'upper':[0, 2, 3, 6 ,7]}

switchMap =  {'lower_left':3, 'lower_right':1, 'upper_left':0, 'upper_right':2}

gripperMap = {'DL':[800, 1600, 1650, 200, 0], 
                   'DR':[2500, 3300, 50, 1350, 1], 
                   'UL':[1900, 2700, 400, 1900, 2], 
                   'UR':[0, 800, 2000, 500, 3]}

armName = ['DL', 'DR', 'UL', 'UR']

stretchDIR =  {'lower_left':-1, 'lower_right':1, 'upper_left':-1, 'upper_right':1}

jointThreshold = {'vertical': 0.015, 'horizontal': 0.03, 'extend': 0.025}
reachedThreshold = {'vertical': 0.01, 'horizontal': 0.01, 'extend': 0.01}

settedTargets = {'lower':[[0.35, -1.2, 1.3, 0],[1.25, -1.2, 1.2, 0],[1.55, -1.2, 1.0, 1]],
                 'upper':[[0.35, -1.2, 1.8, 0],[1.25, -1.2, 1.7, 1],[1.55, -1.2, 1.8, 1]]}