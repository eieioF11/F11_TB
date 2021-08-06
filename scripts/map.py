#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import numpy as np

#Import message type, OccupancyGrid is the message type
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt

import random
import math as m
import rrt_star

def rrt_pathplanner(obstacleList):
    # Set Initial parameters
    rrt = rrt_star.RRT(start=[185,195], goal=[159, 164],randArea=[133,230], obstacleList=obstacleList)
    path = rrt.Planning(animation=True)
    # Draw final path
    rrt.DrawGraph()
    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.grid(True)
    plt.pause(0.01)  # Need for Mac
    plt.show()

class Map(object):
	def __init__(self):
	#rospySubscribe to the map topic, the second is the data type, the third is the callback function
	# Pass the subscribed data to the callback function, which is the mapmsg variable
	#If a topic comes, call the callback function directly
		self.map_sub = rospy.Subscriber("map",OccupancyGrid, self.callback)
		print "get map~"
		#The output below is the address, not the data
		print self.map_sub

	#The definition of the callback function, passed mapmsg
	def callback(self,mapmsg):
		obstacleList = np.empty(3)
		try:
			print "into callback"
			#Mainly want to get data, here is stored map information
			map = mapmsg.data
			# Below is the tuple type
			print type(map)
			#Change to numpy format that can draw pictures
			map = np.array(map)
			#The following output is (368466,), obviously can not draw
			print map.shape
			#Need to reshape, factor the above numbers online, then calculate the two largest factors
			#So it's probably like this:
			map = map.reshape((mapmsg.info.height,mapmsg.info.width))
			print map
			#You can see that most of the values ​​are -1, so you need to regularize the values
			row,col = map.shape
			print row,col
			tem = np.zeros((row,col))
			for i in range(row):
				for j in range(col):
					if(map[i,j]==-1):
						tem[j,i]=255
					else:
						tem[j,i]=map[i,j]
			print map.shape

			for i in range(col):
				for j in range(row):
					if(tem[i,j]==100):
						obstacleList=np.vstack((obstacleList, np.array([i,j,0.15])))
			obstacleList=np.delete(obstacleList, 0, 0)

		except Exception,e:
			print e
			rospy.loginfo('convert rgb image error')

		print obstacleList
		rrt_pathplanner(obstacleList.tolist())

	def getImage():
		return self.rgb_image

def main(_):
	rospy.init_node('map',anonymous=True)
	v=Map()
	rospy.spin()

if __name__=='__main__':
	main('_')