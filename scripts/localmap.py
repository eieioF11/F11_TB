#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math

import rospy
from sensor_msgs.msg import LaserScan

import matplotlib.pyplot as plt

from icp import icp
from Timer import Timer

import cv2

class LocalMap():
    def __init__(self):
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)   #'scan'トピックをLaserScan型で購読し，scan_callback関数を呼び出す
        self.Obstacle=True#障害物フラグ
        self.Range_ahead=0.0
        self.mapsize=150 #マップ配列のサイズ
        self.maprange=1.0#[m] (中心から上下左右にmaprange[m]の範囲)
        self.localmap= np.zeros([self.mapsize,self.mapsize],dtype=np.uint8)
        self.localmap_pre=self.localmap
        self.kernel1 = np.ones((10,10),np.uint8)
        #self.kernel = np.ones((10,10),np.uint8)
        self.kernel=cv2.getStructuringElement(cv2.MORPH_CROSS,(5,5))
        #self.kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        self.t0=Timer()
        self.obstacles=np.zeros((1,2))
        self.obstacles_pre=np.zeros((1,2))

    ####################################
    # Callback for receiving Scan  #
    ####################################

    def obstacle(self):
        return self.Obstacle

    def range_ahead(self):
        return self.Range_ahead

    def scan_callback(self,msg):
        self.Range_ahead = (msg.ranges[0]+msg.ranges[1]+msg.ranges[len(msg.ranges)-2]+msg.ranges[len(msg.ranges)-1])/4#ロボットの真正面にある障害物までの距離
        #if 0.13 < self.Range_ahead and self.Range_ahead <0.18:
        #    self.Obstacle=True
        #else:
        #    self.Obstacle=False
        self.Obstacle=False
        dangle=(msg.angle_max-msg.angle_min)/len(msg.ranges)
        #print(msg.angle_max,msg.angle_min,dangle)
        angle=msg.angle_min
        #self.localmap_pre=self.localmap
        self.localmap_pre = self.localmap.copy().astype("float")
        self.localmap= np.zeros([self.mapsize,self.mapsize],dtype=np.uint8)
        #if self.t0.stand_by(0.5) or len(self.obstacles)<=1:
        self.obstacles_pre=self.obstacles
        self.obstacles=np.zeros((1,2))
        d=0
        for i in msg.ranges:
            #localmap作成
            x=i*math.cos(angle)
            y=i*math.sin(angle)
            if math.fabs(x)<self.maprange and math.fabs(y)<self.maprange:
                self.obstacles=np.append(self.obstacles,[[x,y]], axis=0)
                if x>0 :
                    #x=int((1-x)*(self.mapsize/2.0))
                    #y=int((1-y)*(self.mapsize/2.0))
                    x=int((1-x)*(self.mapsize))
                    y=int((1-y)*(self.mapsize/2.0))
                    self.localmap[x,y]=255
            angle+=dangle
            #障害物検知
            if 0.13 < i and i <0.2:
                d+=1
                #rospy.logwarn("Obstacle detection "+str(i)+"[m]")#正面にある障害物までの距離を表示
        if d>=4:
            self.Obstacle=True
            rospy.logwarn("Obstacle detection "+str(i)+"[m] /"+str(d))
        #transformation_history, aligned_points = icp(self.obstacles_pre,self.obstacles, verbose=False)
        '''
        plt.xlim(-self.maprange,self.maprange)
        plt.ylim(-self.maprange,self.maprange)
        plt.axes().set_aspect('equal')
        plt.plot(self.obstacles[:, 0],self.obstacles[:, 1], 'rx', label='reference points')
        plt.plot(self.obstacles_pre[:, 0],self.obstacles_pre[:, 1], 'b1', label='reference points')
        #plt.plot(aligned_points[:, 0], aligned_points[:, 1], 'g+', label='aligned points')
        plt.pause(0.01)
        plt.clf()           # 画面初期化
        '''

        closing = cv2.morphologyEx(self.localmap, cv2.MORPH_CLOSE, self.kernel)
        closing_pre = cv2.morphologyEx(self.localmap_pre, cv2.MORPH_CLOSE, self.kernel)
        #closing = cv2.dilate(self.localmap, self.kernel, iterations = 1)
        #closing_pre = cv2.dilate(self.localmap_pre, self.kernel, iterations = 1)
        img_color = np.zeros((self.mapsize,self.mapsize, 3),dtype=np.uint8)
        img_color = np.insert(arr=img_color, obj=2, values=closing, axis=2)
        img_color_pre = np.zeros((self.mapsize,self.mapsize, 3),dtype=np.uint8)
        img_color_pre = np.insert(arr=img_color_pre, obj=0, values=closing_pre, axis=2)
        dst = cv2.addWeighted(img_color,0.5,img_color_pre,0.5,0)
        dst = cv2.add(img_color,img_color_pre)
        dst_ = cv2.cvtColor(dst, cv2.COLOR_RGB2BGR)
        black = [0, 0, 0]
        white = [255,255,255]
        color = [255, 0, 255]
        mask = np.zeros((self.mapsize,self.mapsize, 1),dtype=np.uint8)
        mask[np.where((dst_ == color).all(axis=2))]=255
        #dst_[np.where((dst_ == color).all(axis=2))] = black

        mask = cv2.dilate(mask, self.kernel1, iterations = 1)
        mask = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
        dst_ = cv2.add(dst_,mask)
        dst_[np.where((dst_ == white).all(axis=2))] = black

        ksize=3
        #中央値フィルタ
        dst_ = cv2.medianBlur(dst_,ksize)

        #オリジナル画像の高さ・幅を取得
        height = dst.shape[0]
        width = dst.shape[1]
        #リサイズ(拡大/縮小)
        multiple = 4
        dst = cv2.resize(dst , (int(width * multiple), int(height * multiple)))
        dst_ = cv2.resize(dst_ , (int(width * multiple), int(height * multiple)))

        #cv2.imshow("mask",mask)
        #cv2.imshow("Local map",dst)
        #cv2.imshow("Local map_",dst_)
        cv2.waitKey(10)
