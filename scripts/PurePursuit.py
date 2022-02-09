#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
# import for ros function
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path, Odometry

from jsk_rviz_plugins.msg import *
from std_msgs.msg import ColorRGBA, Float32, Int32

from localmap import *
from safezone import *

from enum import Enum

class mode(Enum):
    NORISK  = 0
    SITTING = 1
    STAND   = 2
    WALK    = 3

#######################################
# Simple Path follower (Pure Pursuit) #
#######################################
class Simple_path_follower():

    ##################
    # Initialization #
    ##################
    def __init__(self):

        rospy.init_node('Simple_Path_Follower', anonymous=True)
        self.r = rospy.Rate(50)  # 50hz

        self.target_speed_max = 0.12    #target speed [m/h]
        self.target_speed_min = 0.04
        self.target_LookahedDist = 0.3 #Lookahed distance for Pure Pursuit[m]
        self.maxyawv = 2.0             #[rad/s]
        r=6.00
        self.SZ=SafeZone(r,r-0.1)

        #first flg (for subscribe global path topic)
        self.first=False
        self.path_first_flg = False
        self.odom_first_flg = False
        self.position_search_flg = False
        self.last_indx = 0

        #initialize publisher
        self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)#実機使用時
        #self.cmdvel_pub = rospy.Publisher("/F11Robo/diff_drive_controller/cmd_vel", Twist, queue_size=50)#シュミレーター使用時
        self.lookahed_pub = rospy.Publisher("/lookahed_marker", Marker, queue_size=50)
        self.robo_pub = rospy.Publisher("/robo_marker", Marker, queue_size=50)
        self.value_pub1 = rospy.Publisher("CMD_Vx", Float32, queue_size=1)
        self.value_pub2 = rospy.Publisher("CMD_Az", Float32, queue_size=1)
        self.value_pub3 = rospy.Publisher("Curvature_val", Float32, queue_size=1)
        self.value_pub4 = rospy.Publisher("range_ahead", Float32, queue_size=1)
        self.menu_pub = rospy.Publisher("risk_menu", OverlayMenu, queue_size=1)

        #initialize subscriber
        self.path_sub = rospy.Subscriber("/path", Path, self.cb_get_path_topic_subscriber)
        self.odom_sub = rospy.Subscriber('human',Int32, self.human)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)


        #走行経路のパスを配信
        self.path = Path()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.path_pub = rospy.Publisher('/path_hist', Path, queue_size=10)

        self.cflag=False
        self.target_yaw=0
        self.target_lookahed_x=0
        self.target_lookahed_y=0
        self.oldspeed=0
        self.dist=0
        self.cur_diff=0.0

        self.Lmap=LocalMap()
        self.Obstacle=True#障害物フラグ

        self.safezone=[]
        self.MODE = mode.NORISK

    def map(self,x,in_min,in_max,out_min,out_max):
        value=(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        if value>out_max:
            value=out_max
        if value<out_min:
            value=out_min
        return value

    def human(self,value):
        print(value)
        if value.data==0:
            self.MODE=mode.NORISK
            self.first=True
        elif value.data==1:
            self.MODE=mode.SITTING
        elif value.data==2:
            self.MODE=mode.STAND
        elif value.data==3:
            self.MODE=mode.WALK
            self.first=True
            self.safezone=self.SZ.safezone(self.current_x,self.current_y)


    def publish_robo_marker(self,x,y,yaw_euler):

        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = rospy.Time.now()

        marker_data.ns = "my_name_space"
        marker_data.id = 0

        marker_data.action = Marker.ADD

        marker_data.pose.position.x = x
        marker_data.pose.position.y = y
        marker_data.pose.position.z = 0.0

        temp_quaternion = tf.transformations.quaternion_from_euler(0,0,yaw_euler)

        marker_data.pose.orientation.x = temp_quaternion[0]
        marker_data.pose.orientation.y = temp_quaternion[1]
        marker_data.pose.orientation.z = temp_quaternion[2]
        marker_data.pose.orientation.w = temp_quaternion[3]

        marker_data.color.r = 0.0
        marker_data.color.g = 0.0
        marker_data.color.b = 1.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 0.5
        marker_data.scale.y = 0.05
        marker_data.scale.z = 0.05

        marker_data.lifetime = rospy.Duration()
        marker_data.type = 0

        self.robo_pub.publish(marker_data)


    def publish_lookahed_marker(self,x,y,yaw_euler):

        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = rospy.Time.now()

        marker_data.ns = "my_name_space"
        marker_data.id = 0

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
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 0.1
        marker_data.scale.y = 0.1
        marker_data.scale.z = 0.1

        marker_data.lifetime = rospy.Duration()
        marker_data.type = 0

        self.lookahed_pub.publish(marker_data)

    ###################
    # Update cmd_vel  #
    ###################
    def update_cmd_vel(self):
        self.cb_get_odometry()
        speed=0
        nowCV=0
        yaw_rate = 0.0
        self.Obstacle=self.Lmap.obstacle()
        #if self.MODE==mode.WALK:
        #    if len(self.safezone)>1:
        #        target_lookahed_x=self.safezone[0]
        #        target_lookahed_y=self.safezone[1]
        #        #print(self.safezone)
        #        target_yaw = math.atan2(target_lookahed_y-self.current_y,target_lookahed_x-self.current_x)
        #        self.publish_lookahed_marker(target_lookahed_x,target_lookahed_y,target_yaw)
        menu = OverlayMenu()
        menu.title = "Risk Level"
        #NORISK  = 0
        #SITTING = 1
        #STAND   = 2
        #WALK    = 3
        menu.menus = ["NORISK", "SITTING", "STAND", "WALK"]
        menu.current_index = self.MODE.value
        menu.fg_color.r = 1.0
        menu.fg_color.g = 1.0
        menu.fg_color.b = 1.0
        menu.fg_color.a = 1.0
        menu.bg_color.r = 0.0
        menu.bg_color.g = 0.0
        menu.bg_color.b = 0.0
        menu.bg_color.a = 1.0
        self.menu_pub.publish(menu)
        if self.path_first_flg == True and self.odom_first_flg == True:

            dist_from_current_pos_np = np.sqrt(np.power((self.path_x_np-self.current_x),2) + np.power((self.path_y_np-self.current_y),2))
            min_indx = dist_from_current_pos_np.argmin()
            nearest_x = self.path_x_np[min_indx]
            nearest_y = self.path_y_np[min_indx]
            # Get nearest Path point at first time123
            if self.position_search_flg == False:
                self.pass_flg_np[0:min_indx] = 1    #Set pass flg
                self.position_search_flg = True
            else:
                # Check pass flg from vehicle position
                for indx in range (self.last_indx,self.path_x_np.shape[0]):
                    if dist_from_current_pos_np[indx] < 0.1:
                        self.pass_flg_np[indx] = 1
                    else:
                        break
            self.last_indx = min_indx

            #check goal
            if self.pass_flg_np[self.path_x_np.shape[0]-1] == 1:
                cmd_vel = Twist()
                self.cmdvel_pub.publish(cmd_vel)
                self.path_first_flg = False
                rospy.loginfo("goal!!")
                return
            #calculate target point
            dist_sp_from_nearest = 0.0
            dist_sp_from_nearest_N=0.0
            target_lookahed_x = nearest_x
            target_lookahed_y = nearest_y
            for indx in range (self.last_indx,self.path_x_np.shape[0]):
                dist_sp_from_nearest = self.path_st_np[indx] - self.path_st_np[self.last_indx]
                if indx+1 < self.last_indx:
                    dist_sp_from_nearest_N = self.path_st_np[indx+1] - self.path_st_np[self.last_indx]
                else:
                    dist_sp_from_nearest_N=dist_sp_from_nearest
                tld=math.fabs(self.target_LookahedDist-self.map(self.curvature_val[indx],0,np.amax(self.curvature_val),0,self.target_LookahedDist-0.1))
                #if self.cur_diff>=0.7:
                #    speed=math.fabs(self.target_LookahedDist-self.map(self.curvature_val[indx],0,np.amax(self.curvature_val),self.target_speed_min,self.target_speed_max))
                #else:
                #    speed=self.target_speed_max
                speed=self.target_speed_max
                nowCV=self.curvature_val[indx]#debug
                if tld>=self.target_LookahedDist:
                    tld=self.target_LookahedDist
                if (dist_sp_from_nearest) > tld and not(dist_sp_from_nearest_N > dist_sp_from_nearest):
                    print self.target_LookahedDist,"tld:",tld
                    self.target_lookahed_x = self.path_x_np[indx]
                    self.target_lookahed_y = self.path_y_np[indx]
                    self.cflag=True
                    break
            target_lookahed_x=self.target_lookahed_x
            target_lookahed_y=self.target_lookahed_y
            if self.cflag:
                self.target_yaw = math.atan2(target_lookahed_y-self.current_y,target_lookahed_x-self.current_x)
                self.oldspeed=speed
                self.cflag=False
            else:
                self.dist=math.sqrt((self.target_lookahed_x-self.current_x)**2+(self.target_lookahed_y-self.current_y)**2)
                speed=self.map(self.dist,0,self.target_LookahedDist,0,self.oldspeed)
            target_yaw=self.target_yaw

            if self.MODE==mode.WALK:
                if len(self.safezone)>1:
                    speed=0
                    target_lookahed_x=self.safezone[0]
                    target_lookahed_y=self.safezone[1]
                    target_yaw = math.atan2(target_lookahed_y-self.current_y,target_lookahed_x-self.current_x)
                    self.dist=math.sqrt((target_lookahed_x-self.current_x)**2+(target_lookahed_y-self.current_y)**2)
                    speed=self.map(self.dist,0,self.target_LookahedDist,0,self.oldspeed)*2
                    if self.dist<0.04:
                        speed=0.0
                else:
                    self.first=True
                    speed=0.0

            yaw_diff = target_yaw - self.current_yaw_euler

            if yaw_diff > math.pi:
                yaw_diff = -2*math.pi+yaw_diff
            elif yaw_diff < -math.pi:
                yaw_diff = 2*math.pi+yaw_diff

            sample_sec = dist_sp_from_nearest/(speed)
            #sample_sec = 0.35
            if sample_sec != 0.0:
                yaw_rate = yaw_diff/sample_sec
            else:
                yaw_rate = 0.0

            #if sample_sec != 0.0:
            #    yaw_rate = math.fabs(yaw_diff)/sample_sec
            #else:
            #    yaw_rate = 0.0

            # check vehicle orientation and target yaw
            #if math.fabs(target_yaw - self.current_yaw_euler) < math.pi:
            #    if (target_yaw) < (self.current_yaw_euler):
            #        yaw_rate = yaw_rate * (-1.0)
            #elif math.fabs(target_yaw - self.current_yaw_euler) > math.pi:
            #    if (target_yaw) > (self.current_yaw_euler):
            #        yaw_rate = yaw_rate * (-1.0)
            if self.MODE==mode.WALK:
                yaw_rate*=4.0
            else:
                yaw_rate*=1.7
            max_yawv=self.maxyawv
            if yaw_rate>=max_yawv:
                yaw_rate=max_yawv
            min_yawv=-self.maxyawv
            if yaw_rate<=min_yawv:
                yaw_rate=min_yawv

            if self.MODE==mode.STAND:
                speed=self.target_speed_min

            #Set Cmdvel
            if self.first and math.fabs(yaw_diff)<(math.pi/30):
                self.first=False
            elif not self.first:
                if speed>(self.target_speed_max):
                    speed=self.target_speed_max
                #elif speed<(self.target_speed_min/3.6):
                #    speed=self.target_speed_min/3.6
            else:
                speed=0.0


            #if self.Obstacle:
            #    speed=0.0
            #Set Cmdvel
            cmd_vel = Twist()
            cmd_vel.linear.x = speed    #[m/s]
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = yaw_rate
            self.cmdvel_pub.publish(cmd_vel)

            if not self.Obstacle:
                rospy.loginfo("{},{:.3f}[deg],{:.3f}[deg],{:.3f}[deg],yaw_rate:{:.3f},Vx:{:.3f},dist:{:.3f}".format(self.first,yaw_diff*180/math.pi,target_yaw*180/math.pi,self.current_yaw_euler*180/math.pi,yaw_rate,speed,self.dist))

            #publish maker
            self.publish_lookahed_marker(target_lookahed_x,target_lookahed_y,target_yaw)
            self.publish_robo_marker(self.current_x,self.current_y,self.current_yaw_euler)
            #print("cmd_vel_update")
            self.cur_diff=np.amax(self.curvature_val)-np.amin(self.curvature_val)#曲率最大最小の差
            #debug
        self.value_pub1.publish(speed)
        self.value_pub2.publish(yaw_rate)
        self.value_pub3.publish(nowCV)
        self.value_pub4.publish(self.Lmap.range_ahead())
        self.r.sleep()
        return

    ####################################
    # Callback for receiving Odometry  #
    ####################################

    def odom_cb(self,data):
        if self.path_first_flg:
            self.path.header = data.header
            pose = PoseStamped()
            pose.header = data.header
            pose.pose = data.pose.pose
            self.path.poses.append(pose)
            self.path_pub.publish(self.path)
        else:
            self.path = Path()

    def cb_get_odometry(self):
        try:
            t = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            self.current_x=t.transform.translation.x
            self.current_y=t.transform.translation.y
            e = tf.transformations.euler_from_quaternion((t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w))
            yaw_euler = e[2]
            self.current_yaw_euler = yaw_euler
            if not self.odom_first_flg:
                rospy.loginfo("get odom")
            self.odom_first_flg = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as er:
            print(er)

    ######################################
    # Callback for receiving path topic  #
    ######################################
    def cb_get_path_topic_subscriber(self,msg):
        if self.path_first_flg != True:
            self.path_x_np = np.zeros([len(msg.poses)])
            self.path_y_np = np.zeros([len(msg.poses)])
            self.path_st_np = np.zeros([len(msg.poses)])
            self.pass_flg_np = np.zeros([len(msg.poses)])
            last_x = 0.0
            last_y = 0.0
            last_st = 0.0
            for indx in range(len(msg.poses)):
                self.path_x_np[indx] = msg.poses[indx].pose.position.x
                self.path_y_np[indx] = msg.poses[indx].pose.position.y
                self.path_st_np[indx] = last_st + math.sqrt((self.path_x_np[indx]-last_x)**2 + (self.path_y_np[indx]-last_y)**2)
                last_x = self.path_x_np[indx]
                last_y = self.path_y_np[indx]
                last_st = self.path_st_np[indx]
            x_t = np.gradient(self.path_x_np)
            y_t = np.gradient(self.path_y_np)
            xx_t = np.gradient(x_t)
            yy_t = np.gradient(y_t)
            self.curvature_val = np.abs(xx_t * y_t - x_t * yy_t) / (x_t * x_t + y_t * y_t)**1.5
            print self.curvature_val
            #plt.plot(self.curvature_val)
            #plt.show()
            self.first = self.path_first_flg = True
            rospy.loginfo("get path")

if __name__ == '__main__':
    test = Simple_path_follower()
    rospy.loginfo('Path following is Started...')
    try:
        while not rospy.is_shutdown():
            test.update_cmd_vel()
    except KeyboardInterrupt:
        print("finished!")
