#!/usr/bin/python
# -*- coding:utf-8 -*-
# dance_robot.py
import rospy
from std_msgs.msg import Empty
import pygame
import time

class DanceRobot(object):
	def __init__(self):
		rospy.init_node('dance_robot')
		rospy.Subscriber('/pro/dance_robot', Empty)

	def dance_robot():
		pygame.init()
		pygame.mixer.init()
		pygame.mixer.music.load('/home/tomoya/Music/youkai_split.mp3')
		pygame.mixer.music.play(1)
		# print "音量:%s" % pygame.mixer.music.get_volume() #ボリュームの取得
		# print "再生時間:%s[ms]"%pygame.mixer.music.get_busy() # 再生時間の取得[ms]
		time.sleep(60)
		pygame.mixer.music.stop() # 再生の終了

def main():
	dance_robot = DanceRobot()
	rospy.spin()

if __name__ == '__main__':
	main()
