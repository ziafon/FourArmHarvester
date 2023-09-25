#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import message_filters
from result_msgs.msg import bboxes
import mmap, contextlib
import fusion
import _thread
import time


path = '/home/arl/FourArmHarvester/data/'


class Vision():

    def __init__(self):
        self.cam_name = ['UL', 'UR', 'DL', 'DR']
        self.count = 0
        self.result = {'UL':[[],[],[]], 'UR':[[],[],[]], 'DL':[[],[],[]], 'DR':[[],[],[]]}

        
    def resultSubscribe(self):
        res1 = message_filters.Subscriber('/result2', bboxes)
        res2 = message_filters.Subscriber('/result3', bboxes)
        data = message_filters.ApproximateTimeSynchronizer([res1, res2], queue_size=5, slop=0.5, allow_headerless = True)
        data.registerCallback(self.processing)



    def processing(self, data1, data2):
        # d0,color3,DR,b31; d1:color2,UL,b21; d2,CAM1,DL,b22; d3,CAM2,UR,b32
        with open(path + 'depth1.dat', 'r') as f:
            with contextlib.closing(mmap.mmap(f.fileno(), 480*640*2, access=mmap.ACCESS_READ)) as m:
                data = m.read(480*640*2)
                d0 = np.frombuffer(data, dtype=np.uint16).reshape(480, 640)
        with open(path + 'depth2.dat', 'r') as f:
            with contextlib.closing(mmap.mmap(f.fileno(), 480*640*2, access=mmap.ACCESS_READ)) as m:
                data = m.read(480*640*2)
                d1 = np.frombuffer(data, dtype=np.uint16).reshape(480, 640)
        with open(path + 'depth3.dat', 'r') as f:
            with contextlib.closing(mmap.mmap(f.fileno(), 480*640*2, access=mmap.ACCESS_READ)) as m:
                data = m.read(480*640*2)
                d2 = np.frombuffer(data, dtype=np.uint16).reshape(480, 640)
        with open(path + 'depth4.dat', 'r') as f:
            with contextlib.closing(mmap.mmap(f.fileno(), 480*640*2, access=mmap.ACCESS_READ)) as m:
                data = m.read(480*640*2)
                d3 = np.frombuffer(data, dtype=np.uint16).reshape(480, 640)

        idx = self.count % 3
        self.bbox_generate('UL','DL', d1, d2, data1.bboxes, idx)
        self.bbox_generate('DR','UR', d0, d3, data2.bboxes, idx)
        self.count += 1


    def bbox_generate(self, id1, id2, dep1, dep2, data, idx=0):
        self.result[id1][idx] = []
        self.result[id2][idx] = []
        if data:
            for bbox in data:
                bd = bbox.boundbox
                cx = int((bd[0] + bd[2])/2)
                cy = int((bd[1] + bd[3])/2)
                c = bbox.mask_d
                if cx >= 640 or cx < 0 or cy >= 480 or cy < 0: continue
                if bbox.bbox_d == 1:
                    d = dep1[cy, cx]
                    self.result[id1][idx].append([cx, cy, d, c])
                if bbox.bbox_d == 2:
                    d = dep2[cy, cx]
                    self.result[id2][idx].append([cx, cy, d, c])
        if len(self.result[id1]) == 0: self.result[id1][idx] = [[0, 0, -1, -1]]
        if len(self.result[id2]) == 0: self.result[id2][idx] = [[0, 0, -1, -1]]






















