#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import rospy
import numpy as np
import _thread
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import mmap, contextlib

'''
接收不同源的图像，放入缓存区同一目录下，方便复用。
'''


path = '/home/arl/FourArmHarvester/data/'


bridge = CvBridge()
bridge.imgmsg_to_cv2

size = 640*480

rospy.init_node('multiCamera')


with open(path + 'color1.dat', 'w') as f:
	f.write('\x00'*size*3)

with open(path + 'depth1.dat', 'w') as f:
	f.write('\x00'*size*2)

with open(path + 'color2.dat', 'w') as f:
	f.write('\x00'*size*3)

with open(path + 'depth2.dat', 'w') as f:
	f.write('\x00'*size*2)

with open(path + 'color3.dat', 'w') as f:
	f.write('\x00'*size*3)

with open(path + 'depth3.dat', 'w') as f:
	f.write('\x00'*size*2)

with open(path + 'color4.dat', 'w') as f:
	f.write('\x00'*size*3)

with open(path + 'depth4.dat', 'w') as f:
	f.write('\x00'*size*2)



def colorSubscriber1():
    rospy.Subscriber('/camera1/color/image_raw', Image, callback=color_callback, callback_args=2)
    rospy.spin()

def depthSubscriber1():    
    rospy.Subscriber('/camera1/aligned_depth_to_color/image_raw', Image, callback=depth_callback, callback_args=2)
    rospy.spin()

def colorSubscriber2():
    rospy.Subscriber('/camera2/color/image_raw', Image, callback=color_callback, callback_args=3)
    rospy.spin()

def depthSubscriber2():    
    rospy.Subscriber('/camera2/aligned_depth_to_color/image_raw', Image, callback=depth_callback, callback_args=3)
    rospy.spin()

def colorSubscriber3():
    rospy.Subscriber('/color2', Image, callback=color_callback, callback_args=1)
    rospy.spin()

def depthSubscriber3():
    rospy.Subscriber('/depth2', Image, callback=depth_callback, callback_args=1)
    rospy.spin()

def colorSubscriber4():
    rospy.Subscriber('/color3', Image, callback=color_callback, callback_args=0)
    rospy.spin()

def depthSubscriber4():
    rospy.Subscriber('/depth3', Image, callback=depth_callback, callback_args=0)
    rospy.spin()

# 接收彩色图消息转存
def color_callback(msg, cameraID):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    with open(path + 'color{}.dat'.format(cameraID+1), 'r+') as f:
        with contextlib.closing(mmap.mmap(f.fileno(), 480*640*3, access=mmap.ACCESS_WRITE)) as m:
            m.seek(0)
            m.write(cv_image.tobytes())

# 接收深度图消息转存
def depth_callback(msg, cameraID):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
    with open(path + 'depth{}.dat'.format(cameraID+1), 'r+') as f:
        with contextlib.closing(mmap.mmap(f.fileno(), 480*640*2, access=mmap.ACCESS_WRITE)) as m:
            m.seek(0)
            m.write(cv_image.tobytes())


# 每个话题独立线程
_thread.start_new_thread(colorSubscriber1, ())
_thread.start_new_thread(depthSubscriber1, ())
_thread.start_new_thread(colorSubscriber2, ())
_thread.start_new_thread(depthSubscriber2, ())
_thread.start_new_thread(colorSubscriber3, ())
_thread.start_new_thread(depthSubscriber3, ())
_thread.start_new_thread(colorSubscriber4, ())
_thread.start_new_thread(depthSubscriber4, ())


while not rospy.is_shutdown():
    rospy.sleep(1)










