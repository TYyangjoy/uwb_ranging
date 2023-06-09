# -*- coding: UTF-8 -*-
import socket
import threading
# import time
import sys
import csv
import threading
import serial
import glob, json
import collections
import datetime
from math import sin, cos, radians, sqrt, pi, atan, acos, atan
import numpy as np
import pandas as pd
from scipy import optimize
from scipy.optimize import lsq_linear, root, minimize, least_squares
#from scipy.optimize import lsq_linear
# from smbus import SMBus
from datetime import datetime
# from kalmanfilter import KalmanFilter


def ToA_stage_one(distances_to_anchors, anchor_positions): ##x 和 y 軸是準的 ，z軸在共平面時又大誤差
    distances_to_anchors, anchor_positions = np.array(distances_to_anchors), np.array(anchor_positions)
    if not np.all(distances_to_anchors):
        raise ValueError('Bad uwb connection. distances_to_anchors must never be zero. ' + str(distances_to_anchors))
    anchor_offset = anchor_positions[0]
    anchor_positions = anchor_positions[1:] - anchor_offset
    K = np.sum(np.square(anchor_positions), axis=1)   #ax=1 列加
    squared_distances_to_anchors = np.square(distances_to_anchors)
    squared_distances_to_anchors = (squared_distances_to_anchors - squared_distances_to_anchors[0])[1:]
    b = (K - squared_distances_to_anchors) / 2.
    res = lsq_linear(anchor_positions, b, lsmr_tol='auto', verbose=0)
    return res.x + anchor_offset
def ToA(distances_to_anchors, anchor_num, anchor_positions): ##找較準確的z 
    distances_to_anchors, anchor_positions = np.array(distances_to_anchors), np.array(anchor_positions)
    tag_pos = ToA_stage_one(distances_to_anchors, anchor_positions)
    anc_z_ls_mean = np.mean(np.array([i[2] for i in anchor_positions]) )  
    new_z = (np.array([i[2] for i in anchor_positions]) - anc_z_ls_mean).reshape(anchor_num, 1)
    new_anc_pos = np.concatenate((np.delete(anchor_positions, 2, axis = 1), new_z ), axis=1)
    new_disto_anc = np.sqrt(abs(distances_to_anchors[:]**2 - (tag_pos[0] - new_anc_pos[:,0])**2 - (tag_pos[1] - new_anc_pos[:,1])**2))
    new_z = new_z.reshape(anchor_num,)

    a = (np.sum(new_disto_anc[:]**2) - 3*np.sum(new_z[:]**2))/len(anchor_positions)
    b = (np.sum((new_disto_anc[:]**2) * (new_z[:])) - np.sum(new_z[:]**3))/len(anchor_positions)
    cost = lambda z: np.sum(((z - new_z[:])**4 - 2*(((new_disto_anc[:])*(z - new_z[:]))**2 ) + new_disto_anc[:]**4))/len(anchor_positions) 

    function = lambda z: z**3 - a*z + b
    derivative = lambda z: 3*z**2 - a

    ranges = (slice(0, 3, 0.01), )
    resbrute = optimize.brute(cost, ranges, full_output = True, finish = None)
    if resbrute[0] > 0 or resbrute[0] < -2 :
        resbrute = optimize.brute(cost, ranges, full_output = True, finish = None)
    new_tag_pos = [tag_pos[0],tag_pos[1],resbrute[0]]
    
    return new_tag_pos

def trilateration_3d(anchor_positions, distances):
    def fun(x, anchor_positions, distances):
        return np.linalg.norm(anchor_positions - x, axis=1) - distances

    initial_position = np.mean(anchor_positions, axis=0)  # Use the centroid of anchor positions as the initial guess
    res = least_squares(fun, initial_position, args=(anchor_positions, distances), method='lm')
    return res.x


ser_UWB = serial.Serial('/dev/cu.usbmodem1422401', baudrate = 115200)

while True : 
    dis_queue = []
    rx = ser_UWB.readline().decode('utf-8')
    if(rx != ' ' and rx.find('mc') >= 0):
        dis = rx.split(' ')
        dis_array = np.array([(int(dis[2],16)),(int(dis[3],16)), (int(dis[4],16)), (int(dis[5],16))])/1000.0
        if (0 not in dis_array):
            dis_array = dis_array - 0.6
            anchor_positions =[[13,0.13,1.08],[12.5,7.5,1.1],[1.13,7.44,1.18],[1.13,1.26,1.02]]
            error = (trilateration_3d(anchor_positions, dis_array))
            error1 = (ToA(dis_array,4,anchor_positions))
            print(str(error[0]) + " " + str(error[1]) )
                            
