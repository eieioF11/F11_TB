#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import numpy as np
from std_msgs.msg import ColorRGBA, Float32, Int32

import sys
if sys.version_info[0] == 3:
	import tkinter as tk
	from tkinter import messagebox
else:
	import Tkinter as tk
	import tkMessageBox as messagebox

class risktrans():
	def __init__(self,tki):
		self.tki=tki
		self.rt_pub = rospy.Publisher('human',Int32,queue_size=1)
		self.text = tk.StringVar()
		self.text.set("")
		label1 = tk.Label(self.tki, textvariable=self.text)  #文字ラベル設定
		label1.pack(side="top") # 場所を指定　（top, bottom, left, or right）
	def send(self,value):
		#テキスト
		risk=["NORISK","SITTING","STAND","WALK"]
		self.text.set(risk[value])
		#messagebox.showinfo("info", "送信しました")
		self.rt_pub.publish(value)
	def bt1_click(self):
		self.send(0)
	def bt2_click(self):
		self.send(1)
	def bt3_click(self):
		self.send(2)
	def bt4_click(self):
		self.send(3)

def main():
	# 画面作成
	tki = tk.Tk()
	tki.geometry('300x200') # 画面サイズの設定
	tki.title('Risk Level Transmission') # 画面タイトルの設定
	rt=risktrans(tki)
	# ボタンの作成
	X=50
	Y=30
	btn1 = tk.Button(tki, text='NORISK', command = rt.bt1_click)
	btn1.place(x=X, y=Y) #ボタンを配置する位置の設定
	btn2 = tk.Button(tki, text='SITTING', command = rt.bt2_click)
	btn2.place(x=X, y=Y+40) #ボタンを配置する位置の設定
	btn3 = tk.Button(tki, text='STAND', command = rt.bt3_click)
	btn3.place(x=X, y=Y+40*2) #ボタンを配置する位置の設定
	btn4 = tk.Button(tki, text='WALK', command = rt.bt4_click)
	btn4.place(x=X, y=Y+40*3) #ボタンを配置する位置の設定
	try:
		rospy.loginfo('Risk level transmission is Started...')
		rospy.init_node('Risk_level_transmission',anonymous=True)
		tki.mainloop()
	except KeyboardInterrupt:
		pass
	rospy.loginfo('end')


if __name__=='__main__':
	main()