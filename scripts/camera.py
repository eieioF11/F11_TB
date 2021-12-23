#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pandas as pd
import glob
import os
import math
import numpy as np

global avg
global mlist
global oldm
avg=None
mlist=[]

oldm=[]

def find_index_of_nearest_xy(x_array, y_array, x_point, y_point):
    distance = (y_array-y_point)**2 + (x_array-x_point)**2
    i = np.where(distance==distance.min())
    return i

def save_image(img):
    fpath=os.environ['HOME']+"/catkin_ws/src/F11_TB/images/"
    fname=[]
    for f in glob.glob(fpath+'*.jpg'):
        fname.append(int(os.path.splitext(os.path.basename(f))[0]))
    print(fname)
    n=0
    if len(fname):
        n=max(fname)+1
    cv2.imwrite(fpath+str(n)+".jpg", img)

def save_image_mem(img):
    fpath=os.environ['HOME']+"/catkin_ws/src/F11_TB/images/mem/"
    fname=[]
    for f in glob.glob(fpath+'*.jpg'):
        fname.append(int(os.path.splitext(os.path.basename(f))[0]))
    print(fname)
    n=0
    if len(fname):
        n=max(fname)+1
    cv2.imwrite(fpath+str(n)+".jpg", img)

def save_csv(data):
    # Save CSV path file
    cols = ["x", "y"]
    df = pd.DataFrame(data,columns=cols)
    print(df)
    fpath=os.environ['HOME']+"/catkin_ws/src/F11_TB/csv/"
    fname=[]
    for f in glob.glob(fpath+'*.csv'):
        fname.append(int(os.path.splitext(os.path.basename(f))[0]))
    print(fname)
    n=0
    if len(fname):
        n=max(fname)+1
    df.to_csv(fpath+str(n)+".csv",index=False)

def process_image(msg):
    global avg
    global mlist
    global oldm
    try:
        #画像取得
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        frame2=frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #差分計算
        if avg is None:
            avg = gray.copy().astype("float")
            return
        cv2.accumulateWeighted(gray, avg, 0.3)
        frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(avg))
        #二値化
        thresh = cv2.threshold(frameDelta, 3, 255, cv2.THRESH_BINARY)[1]
        #中央値フィルタ
        ksize=21
        thresh = cv2.medianBlur(thresh,ksize)
        #輪郭検出
        image, contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(frame, contours, -1, color=(0, 0, 255), thickness=2)
        #cv2.drawContours(frame2, contours, -1, color=(0, 0, 255), thickness=-1)
        #max_area = 0
        #target = contours[0]
        #for cnt in contours:
        #    #輪郭の面積を求めてくれるcontourArea
        #    area = cv2.contourArea(cnt)
        #    if max_area < area and area < 10000 and area > 1000:
        #        max_area = area
        #        target = cnt
        #cv2.imshow("image",thresh)

        for i in mlist:
            #print i
            cv2.circle(frame,(i[0],i[1]), 1,(0, 255, 255), 2, 4)

        #重心計算
        if len(contours)>0:
            maxCont=contours[0]
            alim=1500
            # 差分があった点を画面に描く
            nowm=[]
            for target in contours:
                x, y, w, h = cv2.boundingRect(target)
                area = w*h#cv2.contourArea(target)
                if area<alim: continue # 小さな変更点は無視
                #print(area)
                cv2.rectangle(frame2, (x, y), (x+w, y+h),(0, 0, 255), 2)
                mu = cv2.moments(target)
                x,y= int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])
                nowm.append((x,y))
                cv2.circle(frame, (x,y), 4,(0, 255, 0), 2, 4)
                if len(maxCont)<len(target):
                    maxCont=target

            if len(oldm) and  len(nowm):
                r=70
                oldm=np.array(oldm)
                for p in nowm:
                    x=p[0]
                    y=p[1]
                    index=find_index_of_nearest_xy(oldm[:,0],oldm[:,1],x,y)
                    x_=oldm[index,0]
                    y_=oldm[index,1]
                    rad=math.atan2(y-y_,x-x_)
                    #print(x_,y_)
                    cv2.circle(frame2,(x,y), 10, color=(0, 0,255), thickness=-1)
                    cv2.arrowedLine(frame2,(x_,y_), (int(r*math.cos(rad))+x_,int(r*math.sin(rad))+y_), (255,0, 0), thickness=20,tipLength=0.5)
            if len(nowm):
                oldm=nowm

            mlist.append([x,y])

        # 読み込んだ画像の高さと幅を取得
        height = frame.shape[0]
        width = frame.shape[1]
        #表示
        cv2.imshow("thresh",cv2.resize(thresh,(width/2, height/2)))
        cv2.imshow("Frame",cv2.resize(frame,(width/2, height/2)))
        cv2.imshow("Frame2",cv2.resize(frame2,(width/2, height/2)))

        key = cv2.waitKey(1) & 0xff
        if key == ord("s"):
            print("save file")
            save_csv(mlist)
            print("save image")
            save_image(frame)
        elif key == ord("p"):
            print("save image")
            save_image_mem(frame2)
        elif key == ord("r"):
            mlist=[]
    except Exception as err:
        print err

def start_node():
    rospy.init_node('img_proc')
    rospy.loginfo('img_proc node started')
    rospy.Subscriber("kodak/kodak_image_view/output", Image, process_image)
    #rospy.Subscriber("/image_raw", Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass