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
        n=max(fname)
    cv2.imwrite(fpath+"conversion/"+str(n)+".jpg", img)

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
        n=max(fname)
    df.to_csv(fpath+"conversion/"+str(n)+".csv",index=False)



def load_csv():
    fpath=os.environ['HOME']+"/catkin_ws/src/F11_TB/csv/"
    fname=[]
    for f in glob.glob(fpath+'*.csv'):
        fname.append(int(os.path.splitext(os.path.basename(f))[0]))
    #print(fname)
    n=0
    if len(fname):
        n=max(fname)
    df = pd.read_csv(fpath+str(n)+".csv")
    return df

def get_distance(a,b):
    d = math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    return d

def process_image(msg):
    global avg
    global mlist
    try:
        #画像取得
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        image = cv2.imread(os.environ['HOME']+"/map.pgm")
        # 読み込んだ画像の高さと幅を取得
        height = frame.shape[0]
        width = frame.shape[1]
        h = image.shape[0]
        w = image.shape[1]
        df=load_csv()
        mlist=df.values
        #print mlist
        center=[width//2,height//2]
        cv2.circle(frame,(center[0],center[1]), 10,(0,0,255), -1, 4)

        cmlist=[]
        cmlist2=[]
        mapcenter=[w//2,h//2]
        s=5.5
        b=center[0]//s-mapcenter[0]
        bx=b+10
        by=b-2
        print b
        for i in mlist:
            d=int(get_distance(center,(i[0],i[1])))
            dir=center-i
            k=50
            corr=k*(dir/d)
            corr[1]=3.2*corr[1]+70
            #print corr
            cmlist.append((i[0]+int(corr[0]),i[1]+int(corr[1])))

        for i in cmlist:
            cmlist2.append((int((i[0]//s)-bx),int((i[1]//s)-by)))


        for i in cmlist:
            cv2.circle(frame,(i[0],i[1]), 1,(0, 255, 255), 2, 4)
        for i in cmlist2:
            cv2.circle(image,(i[0],i[1]), 1,255, 2, 4)


        #表示
        cv2.imshow("Frame",cv2.resize(frame,(width/2, height/2)))
        cv2.imshow("map",image)

        key = cv2.waitKey(1) & 0xff
        if key == ord("s"):
            print("save csv")
            save_csv(cmlist2)
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