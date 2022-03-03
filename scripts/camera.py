#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import tf
import cv2
import pandas as pd
import glob
import os
import math
import numpy as np

class Human_Detection():

    def __init__(self):
        rospy.init_node('img_proc')
        rospy.loginfo('img_proc node started')
        rospy.Subscriber("kodak/kodak_image_view/output", Image, self.process_image)
        #rospy.Subscriber("/image_raw", Image, self.process_image)
        self.Human_pub = rospy.Publisher("/Human_marker", Marker, queue_size=50)
        self.image_pub = rospy.Publisher('detect_image', Image, queue_size=10)
        self.map_sub = rospy.Subscriber("map",OccupancyGrid, self.map)
        self.avg=None
        self.mlist=[]
        self.oldm=[]
        self.getmap=False
        rospy.spin()
        self.H=[]

	#The definition of the callback function, passed mapmsg
    def map(self,mapmsg):
		#obstacleList = np.empty(3)
        try:
            rospy.loginfo('get map')
            #Mainly want to get data, here is stored map information
            map = mapmsg.data
            # Below is the tuple type
            #Change to numpy format that can draw pictures
            map = np.array(map)
            #The following output is (368466,), obviously can not draw
            #Need to reshape, factor the above numbers online, then calculate the two largest factors
            #So it's probably like this:
            map = map.reshape((mapmsg.info.height,mapmsg.info.width))
            print "resol",mapmsg.info.resolution,"h",mapmsg.info.height,"w",mapmsg.info.width
            print "orizin x:",mapmsg.info.origin.position.x,"y:",mapmsg.info.origin.position.y
            self.ox=mapmsg.info.origin.position.x
            self.oy=mapmsg.info.origin.position.y
            self.index_ox=int(-1*self.ox/mapmsg.info.resolution)
            self.index_oy=int(-1*self.oy/mapmsg.info.resolution)
            self.mapresol=mapmsg.info.resolution
            self.getmap=True
        except Exception,e:
            print e

    def publish_marker(self,x,y,yaw_euler,id,r):

        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = rospy.Time.now()

        marker_data.ns = "my_name_space"
        marker_data.id = id

        marker_data.action = Marker.ADD

        marker_data.pose.position.x = x
        marker_data.pose.position.y = y
        marker_data.pose.position.z = 0.0

        temp_quaternion = tf.transformations.quaternion_from_euler(0,0,yaw_euler)

        marker_data.pose.orientation.x = temp_quaternion[0]
        marker_data.pose.orientation.y = temp_quaternion[1]
        marker_data.pose.orientation.z = temp_quaternion[2]
        marker_data.pose.orientation.w = temp_quaternion[3]

        marker_data.color.r = 1.0
        marker_data.color.g = 1.0
        marker_data.color.b = 0.0
        marker_data.color.a = r

        marker_data.scale.x = 0.1
        marker_data.scale.y = 0.1
        marker_data.scale.z = 0.1

        marker_data.lifetime = rospy.Duration()
        marker_data.type = 0

        self.Human_pub.publish(marker_data)

    def find_index_of_nearest_xy(self,x_array, y_array, x_point, y_point):
        distance = (y_array-y_point)**2 + (x_array-x_point)**2
        i = np.where(distance==distance.min())
        return i

    def save_image(self,img):
        fpath=os.environ['HOME']+"/catkin_ws/src/F11_TB/images/"
        fname=[]
        for f in glob.glob(fpath+'*.jpg'):
            fname.append(int(os.path.splitext(os.path.basename(f))[0]))
        print(fname)
        n=0
        if len(fname):
            n=max(fname)+1
        cv2.imwrite(fpath+str(n)+".jpg", img)

    def save_image_mem(self,img):
        fpath=os.environ['HOME']+"/catkin_ws/src/F11_TB/images/mem/"
        fname=[]
        for f in glob.glob(fpath+'*.jpg'):
            fname.append(int(os.path.splitext(os.path.basename(f))[0]))
        print(fname)
        n=0
        if len(fname):
            n=max(fname)+1
        cv2.imwrite(fpath+str(n)+".jpg", img)

    def save_csv(self,data):
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

    def process_image(self,msg):
        try:
            #画像取得
            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
            map = cv2.imread(os.environ['HOME']+"/map.pgm")
            frame2=frame.copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # 読み込んだ画像の高さと幅を取得
            height = frame.shape[0]
            width = frame.shape[1]
            #差分計算
            if self.avg is None:
                self.avg = gray.copy().astype("float")
                return
            cv2.accumulateWeighted(gray, self.avg, 0.3)
            frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(self.avg))
            #二値化
            thresh = cv2.threshold(frameDelta, 3, 255, cv2.THRESH_BINARY)[1]
            #中央値フィルタ
            #ksize=21
            ksize=7
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

            for i in self.mlist:
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
                try:
                    for i in self.H:
                        id=i[0]
                        Hx=i[1]
                        Hy=i[2]
                        rad=i[3]
                        self.publish_marker(Hx,Hy,rad*-1,id,0.0)
                except:
                    pass
                self.H=[]
                if len(self.oldm) and  len(nowm):
                    self.oldm=np.array(self.oldm)
                    id=0
                    for p in nowm:
                        x=p[0]
                        y=p[1]
                        index=self.find_index_of_nearest_xy(self.oldm[:,0],self.oldm[:,1],x,y)
                        x_=self.oldm[index,0]
                        y_=self.oldm[index,1]
                        dist = ((y_-y)**2 + (x_-x)**2)[0,0]
                        rad=math.atan2(y-y_,x-x_)
                        r=1*dist
                        #print(r)
                        if r>200 or r==0:
                            r=0
                        elif r<20:
                            r=20
                        elif r>100:
                            r=100
                        #print(x_,y_)
                        cv2.circle(frame2,(x,y), 5, color=(0, 0,255), thickness=-1)
                        cv2.arrowedLine(frame2,(x_,y_), (int(r*math.cos(rad))+x_,int(r*math.sin(rad))+y_), (0,255,255), thickness=7,tipLength=0.5)

                        if self.getmap:
                            center=[width//2,height//2]
                            h = map.shape[0]
                            w = map.shape[1]
                            mapcenter=[w//2,h//2]
                            s=5.5
                            b=center[0]//s-mapcenter[0]
                            bx=b+20
                            by=b-2
                            r=0
                            x_=int((x_//s)-bx)
                            y_=int((y_//s)-by)
                            cv2.circle(map,(x_,y_), 1,255, 2, 4)
                            tx=int(x_*math.cos(r)-y_*math.sin(r))
                            ty=int(x_*math.sin(r)+y_*math.cos(r))
                            Hx=(tx-self.index_ox)*self.mapresol-0.2
                            Hy=(ty-self.index_oy)*self.mapresol*-1-0.9
                            print(tx,ty,Hx,Hy)
                            self.publish_marker(Hx,Hy,rad*-1,id,1.0)
                            id+=1
                            self.H.append([id,Hx,Hy,rad*-1])
                    self.mlist.append([x,y])
                if len(nowm):
                    self.oldm=nowm

            #表示
            cv2.imshow("thresh",cv2.resize(thresh,(width/2, height/2)))
            cv2.imshow("Frame",cv2.resize(frame,(width/2, height/2)))
            cv2.imshow("Frame2",cv2.resize(frame2,(width/2, height/2)))
            cv2.imshow("map",map)
            bridge = CvBridge()
            msg = bridge.cv2_to_imgmsg(frame2, encoding="bgr8")
            self.image_pub.publish(msg)

            key = cv2.waitKey(1) & 0xff
            if key == ord("s"):
                print("save file")
                self.save_csv(self.mlist)
                print("save image")
                self.save_image(frame)
                #self.save_image(thresh)
            elif key == ord("p"):
                print("save image")
                self.save_image_mem(frame2)
                self.save_image_mem(thresh)
            elif key == ord("r"):
                self.mlist=[]
        except Exception as err:
            print err

if __name__ == '__main__':
    try:
        HD=Human_Detection()
    except rospy.ROSInterruptException:
        pass