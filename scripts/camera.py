#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pandas as pd
import glob
import os

global avg
global mlist
avg=None
mlist=[]

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
    try:
        #画像取得
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #差分計算
        if avg is None:
            avg = gray.copy().astype("float")
            return
        cv2.accumulateWeighted(gray, avg, 0.5)
        frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(avg))
        #二値化
        thresh = cv2.threshold(frameDelta, 3, 255, cv2.THRESH_BINARY)[1]
        #中央値フィルタ
        ksize=7
        thresh = cv2.medianBlur(thresh,ksize)
        #輪郭検出
        image, contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, color=(0, 0, 255), thickness=2)
        #max_area = 0
        #target = contours[0]
        #for cnt in contours:
        #    #輪郭の面積を求めてくれるcontourArea
        #    area = cv2.contourArea(cnt)
        #    if max_area < area and area < 10000 and area > 1000:
        #        max_area = area
        #        target = cnt
        #cv2.imshow("image",thresh)

        #重心計算
        if len(contours)>0:
            maxCont=contours[0]
            for c in contours:
                if len(maxCont)<len(c):
                    maxCont=c
            mu = cv2.moments(maxCont)
            x,y= int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])
            mlist.append([x,y])
            cv2.circle(frame, (x,y), 4,(0, 255, 0), 2, 4)
            for i in mlist:
                #print i
                cv2.circle(frame,(i[0],i[1]), 4,(0, 255, 255), 2, 4)

        # 読み込んだ画像の高さと幅を取得
        height = frame.shape[0]
        width = frame.shape[1]
        #表示
        cv2.imshow("thresh",cv2.resize(thresh,(width/2, height/2)))
        cv2.imshow("Frame",cv2.resize(frame,(width/2, height/2)))

        key = cv2.waitKey(1) & 0xff
        if key == ord("s"):
            print("save file")
            save_csv(mlist)
            print("save image")
            save_image(frame)
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