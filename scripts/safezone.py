#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import numpy as np
import math

#Import message type, OccupancyGrid is the message type
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import tf2_ros
import matplotlib.pyplot as plt

import pandas as pd
import glob
import os

def load_csv():
    fpath=os.environ['HOME']+"/catkin_ws/src/F11_TB/csv/conversion/"
    fname=[]
    for f in glob.glob(fpath+'*.csv'):
        fname.append(int(os.path.splitext(os.path.basename(f))[0]))
    #print(fname)
    n=0
    if len(fname):
        n=max(fname)
    df = pd.read_csv(fpath+str(n)+".csv")
    return df.values

#Map
class SafeZone(object):
	def __init__(self,r1,r2):
		self.r1=r1
		self.r2=r2
		self.map_sub = rospy.Subscriber("map",OccupancyGrid, self.callback)
		print "get map~"
		#The output below is the address, not the data
		print self.map_sub

	def p_trans(self,p,ox,oy,resolution):#ROSにpath形式のデータを配信
		return [(p[0]-ox)*resolution,(p[1]-oy)*resolution]

	def safezone_search(self,inix,iniy,r1,r2,wp,hp):
			#rr=0.1
		#try:
			print(len(wp))
			print(len(hp))
			print(wp)
			print(inix+r1+r2)
			print(inix-(r1+r2))
			print(iniy+r1+r2)
			print(iniy-(r1+r2))
			wp=wp[wp[:,0]<(inix+r1+r2)]
			wp=wp[wp[:,0]>(inix-(r1+r2))]
			wp=wp[wp[:,1]<(iniy+r1+r2)]
			wp=wp[wp[:,1]>(iniy-(r1+r2))]
			print(wp)
			hp=hp[hp[:,0]<(inix+r1+r2)]
			hp=hp[hp[:,0]>(inix-(r1+r2))]
			hp=hp[hp[:,1]<(iniy+r1+r2)]
			hp=hp[hp[:,1]>(iniy-(r1+r2))]
			print(hp)
			print(len(wp))
			print(len(hp))

			theta = np.linspace(0, 2*np.pi, 100)
			x1 = np.array(r1*np.cos(theta)+inix)
			y1 = np.array(r1*np.sin(theta)+iniy)
			p1=np.flipud(np.array(list(zip(x1,y1))))
			#p1=np.round(p1)
			#p1=p1.astype(np.int64)
			p1_o=p1
			result = []
			for line in p1.tolist():
				if line not in result:
					if not self.maph[int(round(line[0])),int(round(line[1]))]:
						result.append(line)
			p1=np.array(result)

			p1_mem=[]
			#障害物と近い場所を削除
			#for j in range(len(p1)):
			#	d=[]#距離のリスト
			#	mind=0
			#	for i in wp:
			#		dir=np.linalg.norm(p1[j]-i)#障害物との距離
			#		d.append(dir)
			#		if dir == min(d):#一番距離が近いとき
			#			mind=dir
			#	#if mind>r2:
			#	#	p1_mem.append(p1[j])
			#	if mind>rr:
			#		p1_mem.append(p1[j])
			#	#print(p1[j],mind)
			#if len(p1_mem)>1:
			#	p1=np.array(p1_mem)

			#safezoneの探索
			safezone=[]
			for j in range(len(p1)):
				d=[]#距離のリスト
				mind=0
				for i in hp:
					dir=np.linalg.norm(p1[j]-i)#人のよく通る場所との距離
					d.append(dir)
					if dir == min(d):#一番距離が近いとき
						mind=dir
				if mind>r2:
					safezone=p1[j]
					break
				#print(p1[j],mind)
			#if len(safezone)>1:
			#	#draw_circle = plt.Circle((safezone[1],safezone[0]),rr,fill=False)
			#	#plt.gcf().gca().add_artist(draw_circle)
			#	draw_circle = plt.Circle((safezone[1],safezone[0]),r2,fill=False)
			#	plt.gcf().gca().add_artist(draw_circle)
			#plt.imshow(self.maph)
			#plt.plot(iniy,inix,"o",c="b")
			#plt.plot(p1_o[:,1],p1_o[:,0],"+",c="r")
			#plt.plot(p1[:,1],p1[:,0],"+",c="c")
			#if len(hp)>1:
			#	plt.plot(hp[:,1],hp[:,0],"+",c="r")
			#if len(wp)>1:
			#	plt.plot(wp[:,1],wp[:,0],"+",c="r")
			#if len(safezone)>1:
			#	plt.plot(safezone[1],safezone[0],"*",markersize=10,c="greenyellow")
			#plt.show()
			return safezone
		#except:
		#	return []

	def safezone(self,x,y):
		szone=[]
		inix=int((x/self.resolution+self.index_ox))
		iniy=int((y/self.resolution+self.index_oy))
		figure, axes = plt.subplots()
		#draw_circle = plt.Circle((iniy,inix),self.r1,fill=False)
		#plt.gcf().gca().add_artist(draw_circle)
		#plt.imshow(self.maph)
		#plt.plot(iniy,inix,"o",c="b")
		#plt.show()
		sz=self.safezone_search(inix,iniy,self.r1,self.r2,self.wp,self.hp)
		if len(sz)>1:
			#plt.imshow(self.maph)
			#plt.plot(sz[1],sz[0],"*",c="greenyellow")
			#plt.plot(iniy,inix,"o",c="b")
			#plt.show()
			print(sz)
			szone=self.p_trans(sz,self.index_ox,self.index_oy,self.resolution)
		else:
			rospy.logerr('Safezone not found')
		return szone

	#The definition of the callback function, passed mapmsg
	def callback(self,mapmsg):
		#obstacleList = np.empty(3)
		try:
			print "into callback"
			#Mainly want to get data, here is stored map information
			map = mapmsg.data
			# Below is the tuple type
			#Change to numpy format that can draw pictures
			map = np.array(map)
			#The following output is (368466,), obviously can not draw
			#Need to reshape, factor the above numbers online, then calculate the two largest factors
			#So it's probably like this:
			map = map.reshape((mapmsg.info.height,mapmsg.info.width))
			#You can see that most of the values ​​are -1, so you need to regularize the values
			row,col = map.shape
			tem = np.zeros((row,col))
			grid = np.zeros((row,col))
			grid_h = np.zeros((row,col))
			for i in range(row):
				for j in range(col):
					if(map[i,j]==-1):
						tem[j,i]=255
					else:
						tem[j,i]=map[i,j]
			#cost map作成
			#n=1
			#rn=3+2*(n-1)
			#print rn
			#for i in range(col):
			#	for j in range(row):
			#		if tem[i,j]==100:
			#			for k in range(rn):
			#				for l in range(rn):
			#					val=tem[i-n+k,j-n+l]
			#					if val==0:
			#						tem[i-n+k,j-n+l]=200
			#経路計算用のマップ作成
			for i in range(col):
				for j in range(row):
					grid[i,j]=0
					if tem[i,j]>0 and tem[i,j]!=255:
						grid[i,j]=1
						grid_h[i,j]=1

			print "resol",mapmsg.info.resolution,"h",mapmsg.info.height,"w",mapmsg.info.width
			print "orizin x:",mapmsg.info.origin.position.x,"y:",mapmsg.info.origin.position.y
			self.ox=mapmsg.info.origin.position.x
			self.oy=mapmsg.info.origin.position.y
			self.resolution=mapmsg.info.resolution
			self.index_ox=int(-1*self.ox/mapmsg.info.resolution)
			self.index_oy=int(-1*self.oy/mapmsg.info.resolution)
			mlist=load_csv()
			print(mlist)
			for i in mlist:
				r=math.pi/2
				x=int(i[0]*math.cos(r)-i[1]*math.sin(r))
				y=int(i[0]*math.sin(r)+i[1]*math.cos(r))
				try :
					if grid_h[y,x]==0:
						grid_h[y,x]=2
				except:
					pass
		except Exception,e:
			print e
			rospy.loginfo('convert rgb image error')
		#tem[int(-1*self.ox/mapmsg.info.resolution),int(-1*self.oy/mapmsg.info.resolution)]=-255
		#plt.imshow(grid_h)
		#plt.show()
		self.maph=grid_h
		wx,wy=np.where(grid_h==1)
		self.wp=np.flipud(np.array(list(zip(wx,wy))))#障害物のリスト
		hx,hy=np.where(grid_h==2)
		self.hp=np.flipud(np.array(list(zip(hx,hy))))#人の通る場所のリスト
		rospy.loginfo('Map conversion completed')