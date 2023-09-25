#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PointStamped
from config import _JointName, motorParam, jointThreshold


cv = motorParam['vertical']
ch = motorParam['horizontal']
ce = motorParam['extend']

tv = jointThreshold['vertical']
th = jointThreshold['horizontal']
te = jointThreshold['extend']


pulse2meter = np.array([cv, cv, ch, ch, ch, ch, ce, ce, ce, ce])
motorThreshold = np.array([tv, tv, th, th, th, th, te, te, te, te])
maxVelocity = np.array([20, 20, 20, 20, 20, 20, 15, 15, 15, 15]) * 100000

a = {'DL':1.00, 'DR':1.00, 'UL':1.00, 'UR':1.00}
b = {'DL':0.55, 'DR':0.55, 'UL':0.55, 'UR':0.55}
oz = np.tan(np.pi/6)
oy = np.cos(np.pi/6)


def adaptiveControl(delta, threshold=motorThreshold):
    """
    根据给定的delta和threshold值计算自适应速度控制。
    
    参数:
        delta (float): Delta值。
        threshold (float): Threshold值。
        
    返回:
        float: 计算出的速度。
    """
    deltaABS = np.abs(delta)
    direction = np.sign(delta)
    # velocity = np.floor(np.exp(deltaABS-threshold)*0.5) * 1500000 + np.sqrt(deltaABS) * 1000000
    velocity = np.floor(np.power(np.e/2, deltaABS-threshold)) * 1500000 + np.sqrt(deltaABS) * 1000000
    return -direction*velocity


def single_adaptive(delta, idx):
    # print(delta)
    deltaABS = np.abs(delta)
    direction = np.sign(delta)
    # velocity = np.floor(np.exp(deltaABS-threshold)*0.5) * 1500000 + np.sqrt(deltaABS) * 1000000
    velocity = np.floor(np.power(np.e/2, deltaABS-motorThreshold[idx])) * 1500000 + np.sqrt(deltaABS) * 1000000
    return -direction*velocity


def unitConvertion(jointPosition, motorPosition):
    motorPosition = np.asarray(motorPosition) / pulse2meter 
    jointPosition[0] = motorPosition[1]
    jointPosition[1] = motorPosition[5]
    jointPosition[2] = motorPosition[9]
    jointPosition[4] = motorPosition[4]
    jointPosition[5] = -motorPosition[8]
    jointPosition[7] = motorPosition[0]
    jointPosition[8] = -motorPosition[2]
    jointPosition[9] = motorPosition[6]
    jointPosition[11] = -motorPosition[3]
    jointPosition[12] = motorPosition[7]
    return jointPosition, motorPosition


def tf_base2fake_listener(arm):
    tf_listener = [tf.TransformListener() for _ in range(4)]
    try:
        for i, v in enumerate(arm):
            tf_listener[i].waitForTransform("fake_{}_link".format(v), 'base_link', rospy.Time(0), rospy.Duration(1))
        print(nowTime(),'tf_base2fake ok')
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        print(nowTime(),'tf_base2fake err')
    return tf_listener


def tf_cam2fake_listener(arm):
    '''
    fake是虚拟的位置，在机械臂的原点。
    '''
    '''
    功能：将相机坐标系转换为手臂坐标系
    输入:
    输出：
    '''
    
    tf_listener = [tf.TransformListener() for i in range(4)]
    try:
        for i, v in enumerate(arm):
            tf_listener[i].waitForTransform("fake_{}_link".format(v), 'camera_{}_link'.format(v), rospy.Time(0), rospy.Duration(1))
        print(nowTime(),'tf_cam2fake ok')
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        print(nowTime(),'tf_cam2fake err')
    return tf_listener



def invCalculate(coordinate, tf_listener, id, dir):
    point = PointStamped()
    point.header.frame_id = 'base_link'
    point.header.stamp = rospy.Time(0)
    point.point.x = coordinate[0]
    point.point.y = coordinate[1]
    point.point.z = coordinate[2]
    trans_to_fake = tf_listener.transformPoint("fake_{}_link".format(id), point)
    fake_x = trans_to_fake.point.x
    fake_y = trans_to_fake.point.y
    fake_z = trans_to_fake.point.z
    fake_z = fake_z - fake_y * oz
    fake_y = fake_y / oy 
    if id == 'DL' or id == 'DR':
        fake_z = np.maximum(fake_z, 0)
    if id == 'UL' or id == 'UR':
        fake_z = np.minimum(fake_z, 0)
    if id == 'DL' or id == 'UL':
        fake_x = np.minimum(fake_x, 0)
    if id == 'DR' or id == 'UR':
        fake_x = np.maximum(fake_x, 0)
    fake_y = np.minimum(fake_y, a[id])
    fake_y = np.maximum(fake_y, b[id])
    return np.array([fake_z, fake_x, dir*(fake_y-b[id])])



def tf_cam2base_listener(cam_id):
    tf_listener = [tf.TransformListener() for i in range(4)]
    try:
        for i in range(4):
            tf_listener[i].waitForTransform('base_link', 'camera_{}_link'.format(cam_id[i]), rospy.Time(0), rospy.Duration(1))
        print(nowTime(),'tf_cam2base ok')
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        print(nowTime(),'tf_cam2base err')
    return tf_listener



def cam2base(tf_trans, j, p):
    point = PointStamped()
    point.header.frame_id = 'camera_{}_link'.format(j)
    point.header.stamp = rospy.Time(0)
    point.point.x = p[2]
    point.point.y = -p[0]
    point.point.z = -p[1]
    trans_in_base = tf_trans.transformPoint('base_link', point)
    # print(arm_name[i]+'_transform to base is :   ', trans_in_base)
    x = trans_in_base.point.x
    y = trans_in_base.point.y
    z = trans_in_base.point.z
    return np.array([x, y, z])


def base2end(tf_trans, j, p):
    point = PointStamped()
    point.header.frame_id = 'baselink'
    point.header.stamp = rospy.Time(0)
    point.point.x = p[2]
    point.point.y = -p[0]
    point.point.z = -p[1]
    trans_in_base = tf_trans.transformPoint('base_link', point)
    # print(arm_name[i]+'_transform to base is :   ', trans_in_base)
    x = trans_in_base.point.x
    y = trans_in_base.point.y
    z = trans_in_base.point.z
    return np.array([x, y, z])


def distanceCalculate(joint_position):
    return [(joint_position[0] - joint_position[7]), 
            (joint_position[4] - joint_position[1]), 
            (joint_position[8] - joint_position[11])]


def is_reached(current, desire):
    tmp = [False, False]
    dis3D = np.sqrt(np.sum(np.square(current-desire)))
    disZ = np.abs(current[2]-desire[2])
    if dis3D < 0.026: 
        tmp[0] = True
    if disZ < 0.015:
        tmp[1] = True
    return tmp, dis3D, disZ


def adaptive(delta, threshold, ratio):
    # print(delta)
    deltaABS = np.abs(delta)
    direction = np.sign(delta)
    # velocity = np.floor(np.exp(deltaABS-threshold)*0.5) * 1500000 + np.sqrt(deltaABS) * 1000000
    velocity = np.floor(np.power(np.e/2, deltaABS-threshold)) * maxVelocity * ratio + np.sqrt(deltaABS) * 1000000
    return -direction*velocity



def nowTime():
    return time.strftime("[%Y-%m-%d %H:%M:%S]", time.localtime(time.time()))


# def plan():




