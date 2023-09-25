#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from itertools import count
from pickle import NONE
import numpy as np

Hstart = 0.95
Hlen = 2.150 - 0.95
Wstart = 0.150
Wlen = 1.781 - 0.150
horizontal_protection = 0.2 # 水平方向双臂碰撞保护距离
vertical_protection = 0.4 # 垂直方向双臂碰撞保护距离

ud = 1.45

DT = {'WD':0.2, 'WU':0.8, 'HL':0.30, 'HR':0.70}

lenFactor  = {
    'E1': [0.00,       DT['HL'],   0.00,       DT['WD']],
    'E2': [0.00,       DT['HL'],   DT['WU'],   1.00],
    'E3': [DT['HR'],   1.00,       DT['WU'],   1.00],
    'E4': [DT['HR'],   1.00,       0.00,       DT['WD']],
    'OC': [DT['HL'],   DT['HR'],   DT['WD'],   DT['WU']],
    'OL': [0.00,       DT['HL'],   DT['WD'],   DT['WU']],
    'OU': [DT['HL'],   DT['HR'],   DT['WU'],   1.00],
    'OR': [DT['HR'],   1.00,       DT['WD'],   DT['WU']],
    'OD': [DT['HL'],   DT['HR'],   0.00,       DT['WD']]
}

def zone_divide(location):

    OBJ_ = {
            'E1': [],'E2':[],'E3':[],'E4': [],
            'OU': [],'OD':[],'OL':[],'OR':[], 'OC':[]   
        }

    n = location.shape[0]
    Wnorm = (location[:, 0] - Wstart) / Wlen
    Hnorm = (location[:, 2] - Hstart) / Hlen

    for i in range(n):
        for key, factor in lenFactor.items():
            if Wnorm[i]>factor[0] and Wnorm[i]<factor[1] and Hnorm[i]>factor[2] and Hnorm[i]<factor[3]:
                OBJ_[key].append(location[i])
    
    return OBJ_



def simple_plan(objects, Targets):
    idx  = np.where(objects[:, 0]>0.5)
    objects = objects[idx]
    idx  = np.where(objects[:, 0]<1.45)
    objects = objects[idx]
    idx = np.argsort(objects[:, 0])
    objects = objects[idx]
    idx = np.where(objects[:,2] < ud)[0]
    down = objects[idx]
    up = np.delete(objects, idx, axis=0)
    while len(down)>0:
        arm = 0
        Targets['lower'].append(np.append(down[0], arm))
        down = np.delete(down, 0, 0)
        if len(down)>0:
            arm = 1 - arm
            Targets['lower'].append(np.append(down[-1], arm))
            down = np.delete(down, -1, 0)
    while len(up)>0:
        arm = 0
        Targets['upper'].append(np.append(up[0], arm))
        up = np.delete(up, 0, 0)
        if len(up)>0:
            arm = 1 - arm
            Targets['upper'].append(np.append(up[-1], arm))
            up = np.delete(up, -1, 0)
    return Targets


def improve_plan(objects, Targets):
    turn = 0
    for obj in objects['E1']:
        Targets['DL'].append(obj)
    for obj in objects['E2']:
        Targets['UL'].append(obj)
    for obj in objects['E3']:
        Targets['UR'].append(obj)
    for obj in objects['E4']:
        Targets['DR'].append(obj)
    if objects['OD']:
        objs = np.asarray(objects['OD'])
        idx = np.argsort(objs[:, 0])
        sort = objs[idx]
        for i in range(sort.shape[0]):
            if turn == 0:
                Targets['DL'].append(np.append(sort[0], 0))
                sort = np.delete(sort, 0, 0)
            if  turn == 1:
                Targets['DR'].append(np.append(sort[-1], 0))
                sort = np.delete(sort, -1, 0)
            turn = 1 - turn
    if objects['OU']:
        objs = np.asarray(objects['OU'])
        idx = np.argsort(objs[:, 0])
        sort = objs[idx]
        for i in range(sort.shape[0]):
            if turn == 0:
                Targets['UL'].append(np.append(sort[0], 0))
                sort = np.delete(sort, 0, 0)
            if  turn == 1: 
                Targets['UR'].append(np.append(sort[-1], 0))
                sort = np.delete(sort, -1, 0)
            turn = 1 - turn
    # for obj in objects['OD']:
    #     if turn == 0:
    #         Targets['DL'].append(obj)
    #     if  turn == 1:
    #         Targets['DR'].append(obj)
    #     turn = 1 - turn
    # for obj in objects['OU']:
    #     if turn == 0:
    #         Targets['UL'].append(obj)
    #     if  turn == 1:
    #         Targets['UR'].append(obj)
    #     turn = 1 - turn
    for obj in objects['OL']:
        if obj[2] > 1.55:
            Targets['UL'].append(obj)
        else:
            Targets['DL'].append(obj)
    for obj in objects['OR']:
        if obj[2] > 1.55:
            Targets['UR'].append(obj)
        else:
            Targets['DR'].append(obj)
    turnU, turnD = 0, 0
    for obj in objects['OC']:
        if obj[2] < 1.55:
            if turnD == 0: Targets['DL'].append(obj)
            else: Targets['DR'].append(obj)
            turnD = 1 - turnD
        else:
            if turnU == 0: Targets['UL'].append(obj)
            else: Targets['UR'].append(obj)
            turnU = 1 - turnU
    for key in Targets:
        Targets[key].append(np.zeros(4))
    return Targets 



def improve_plan_plus(objects, Targets):
    for obj in objects['E1']:
        Targets['DL'].append(obj)
    for obj in objects['E2']:
        Targets['UL'].append(obj)
    for obj in objects['E3']:
        Targets['UR'].append(obj)
    for obj in objects['E4']:
        Targets['DR'].append(obj)
    for obj in objects['OL']:
        if obj[2] > 1.45:
            Targets['UL'].append(obj)
        else:
            Targets['DL'].append(obj)
    for obj in objects['OR']:
        if obj[2] > 1.45:
            Targets['UR'].append(obj)
        else:
            Targets['DR'].append(obj)
    for obj in objects['OC']:
        if obj[2] < 1.45:
            objects['OD'].append(obj)
        else:
            objects['OU'].append(obj)
    objects['OD'].sort(key=lambda x: x[0])
    while not len(objects['OD']) == 0:
        if len(Targets['DL']) < len(Targets['DR']):
            Targets['DL'].append(objects['OD'][0])
            objects['OD'].pop(0)
        else:
            Targets['DR'].append(objects['OD'][-1])
            objects['OD'].pop(-1)
    objects['OU'].sort(key=lambda x: x[0])
    while not len(objects['OU']) == 0:
        if len(Targets['UL']) < len(Targets['UR']):
            Targets['UL'].append(objects['OU'][0])
            objects['OU'].pop(0)
        else:
            Targets['UR'].append(objects['OU'][-1])
            objects['OU'].pop(-1)
    center = [Wstart + Wlen/2, Hstart + Hlen/2]
    for key in Targets:
        if len(Targets[key]) > 0:
            tmp = np.asarray(Targets[key]).reshape(-1,3)
            # idx = np.argsort(np.square(tmp[:,0] - center[0]) + np.square(tmp[:,2] - center[1]))
            idx = np.argsort(np.square(tmp[:,2] - center[1]))
            tmp = np.flip(tmp[idx], axis=0)
            Targets[key] = np.vstack((tmp, np.zeros(3)))
        else:
            Targets[key] = np.zeros((1,3))
    return Targets 






    










