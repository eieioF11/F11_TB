#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


model = cv2.bgsegm.createBackgroundSubtractorMOG()

# 背景差分の面積のしきい値
BOX_VALID_PX_RATIO = 1 / 100
# 使用する背景差分のクラス
BG_SUBTRACTOR_CLASS = cv2.bgsegm.createBackgroundSubtractorMOG
# 検出できなかった場合の学習率
LEARNING_RATE_NOT_DETECTED = 0.0003  #（少し試行した結果、変更可）

def process_image(msg):
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        # 読み込んだ画像の高さと幅を取得
        height = frame.shape[0]
        width = frame.shape[1]

        mask = model.apply(frame)
        cv2.imshow("Frame",cv2.resize(frame,(width/2, height/2)))

        # 背景の画素は黒 (0, 0, 0) にする。
        frame[mask == 0] = 0
        cv2.imshow("Frame (Only Forground)",cv2.resize(frame,(width/2, height/2)))
        cv2.imshow("Mask", cv2.resize(mask,(width/2, height/2)))


        cv2.waitKey(1)
    except Exception as err:
        print err

def start_node():
    rospy.init_node('img_proc')
    rospy.loginfo('img_proc node started')
    #rospy.Subscriber("kodak/kodak_image_view/output", Image, process_image)
    rospy.Subscriber("/image_raw", Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass