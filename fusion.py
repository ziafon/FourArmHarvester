#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, Point32
import tf


# paramDict = paramLoad()

gf = [30, 0.1] # 0:pixel, 1:space
df = [-1.4, -0.5] # 0:min, 1:max

# 内参矩阵
Ki = np.array([[597.267, 0, 319.115],
                [0, 597.267, 234.382],
                [0, 0, 1]])

inv_K =np.linalg.inv(Ki)


def tf_cam2base_listener(cam_id):
    tf_listener = [tf.TransformListener() for i in range(4)]
    try:
        for i in range(4):
            tf_listener[i].waitForTransform('base_link', 'camera_{}_link'.format(cam_id[i]), rospy.Time(0), rospy.Duration(1))
        print('tf_cam2base ok')
        # rospy.loginfo('tf_cam2base ok')
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        # rospy.logerr('tf_cam2base err')
        print('tf_cam2base err')
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
    return [x, y, z]


def splicing(lists):
    arr = []
    for list in lists:
        if len(list)==0:
            continue
        if arr == []: 
            arr = np.asarray(list)
        else:
            arr = np.vstack((arr, np.asarray(list))) 
    return arr.shape((-1, 4))
        

def bbox_filter(arr):
    idx = np.where(arr[:, 2] < 10000)[0]
    arr = arr[idx]
    obj = []
    groups = []
    while len(arr) != 0:
        dis = np.sqrt(np.sum(np.square(arr[0, 0:2] - arr[:, 0:2]), axis=1))
        idx = np.where(dis<gf[0])[0]
        groups.append(arr[idx])
        arr = np.delete(arr, idx, axis=0)
    for group in groups:
        harvest = np.where(group[:, 3]==0)[0]
        tmp = np.median(group, axis=0)
        if len(harvest)>0: tmp[3] = 1
        else: tmp[3] = 2
        if obj == []:
            obj = tmp 
        else: 
            obj = np.vstack((obj, tmp))
    return obj

    
def location_filter(loc):
    # print('to filt', loc)
    cnt = []
    groups = []
    # print(loc)
    while len(loc) != 0:
        dis = np.sqrt(np.sum(np.square(loc[0] - loc), axis=1))
        idx = np.where(dis<gf[1])[0]
        groups.append(loc[idx])
        loc = np.delete(loc, idx, axis=0)
    # print(groups)
    for group in groups:
        harvest = np.where(group[:, 3]==1)[0]
        tmp = np.median(group, axis=0)
        if len(harvest)>0: tmp[3] = 1
        else: tmp[3] = 2
        if cnt == []:
            cnt = tmp
        else:
            cnt = np.vstack((cnt, tmp))
    return cnt


def coordinate_unification(mode, id, obj, tf_trans):
    trans = []
    obj = obj.reshape(-1, 4)
    if mode == 'center':
        n = obj.shape[0]
        for i in range(n):        
            loc = np.dot(inv_K, obj[i,2] * np.array([obj[i,0], obj[i,1], 1]))
            cord = cam2base(tf_trans, id, loc/1000)
            trans.append(np.append(cord, obj[i,3]))
    if mode == 'mask':
        n = obj.shape[0]
        for i in range(n):        
            loc = np.dot(inv_K, obj[i,2] * np.array([obj[i,0], obj[i,1], 1]))
            cord = cam2base(tf_trans, id, loc/1000)
            trans.append(np.append(cord, obj[i,3]))
    return np.asarray(trans).reshape((-1, 4))
    

def fusion(mode, cam_id, obj_list, tf_listener):  
    obj_array = []
    for id in cam_id:
        j = cam_id.index(id)
        # out = splicing(obj_list[id]) 
        out = np.asarray(obj_list[id]).reshape((-1, 4))
        if len(out)==0: 
            continue
        else:
            out = bbox_filter(out)
            if len(out) > 0:
                # print('{} filted bbox'.format(id), out)
                out = coordinate_unification(mode, id, out, tf_listener[j])
                # print('{} transed obj'.format(id), out)
                if len(obj_array) == 0:
                    obj_array = out
                else:
                    obj_array = np.vstack((obj_array, out))
    if len(obj_array) == 0: 
        return obj_array
    else:
        obj_array = obj_array.reshape(-1, 4)
        idx = np.where(obj_array[:,1] > df[0])[0]
        obj_array = obj_array[idx]
        idx = np.where(obj_array[:,1] < df[1])[0]
        obj_array = obj_array[idx]
        return location_filter(obj_array)
        



