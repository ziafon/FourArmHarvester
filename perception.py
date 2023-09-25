#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from vision import Vision
import fusion
import _thread
from robot_msgs.msg import messages


class Preception():

    def __init__(self):
        rospy.init_node('robot_vision')
        self.cam_name = ['UL', 'UR', 'DL', 'DR']
        self.pub = rospy.Publisher('robot_command', messages, queue_size=1)
        self.vision = Vision()
        self.tf_listener = fusion.tf_cam2base_listener(self.cam_name)
        _thread.start_new_thread(self.vision.resultSubscribe, ())
        rospy.Subscriber('robot_command', messages, callback=self.callback,  queue_size=3)
        rospy.spin()


    def callback(self, msgs):
        if msgs.command == 'getDistribution':
            res = {'UL':[], 'UR':[], 'DL':[], 'DR':[]}
            print(self.vision.result)
            for key in self.vision.result:
                for i in range(3):
                    res[key].extend(self.vision.result[key][i]) 
            _distribution = fusion.fusion('center', self.cam_name, res, self.tf_listener)
            print(_distribution)
            data = messages()
            data.command = 'useDection'
            data.objects = _distribution.flatten().tolist()
            self.pub.publish(data)


if __name__=='__main__': 
    try:
        Preception()
    except KeyboardInterrupt:
        rospy.loginfo('Hasta la Vista...')
            







