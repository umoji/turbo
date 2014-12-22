#!/usr/bin/env python
# -*- coding: utf-8 -*-
# test_pro_demo.py

import rospy
import os
import sys

from std_msgs.msg import (
        String,
        Empty,
        Bool,
        )

class ProDemo(object):
    def __init__(self):
        rospy.init_node('pro_demo')
        # publishers
        self.pub_speak_robot = rospy.Publisher('/pro/speak_robot', String)
        self.pub_take_photo = rospy.Publisher('/pro/take_photo', Empty)
        self.pub_catch_drink = rospy.Publisher('/pro/catch_drink', Empty)
        self.pub_dance_robot = rospy.Publisher('/pro/dance_robot', Empty)
        # subscribers
        rospy.Subscriber('/pro/detect_face', Bool, self.cb_detect_face)
        rospy.Subscriber('/pro/get_cmdline_input', String, self.cb_get_cmdline_input)
        rospy.Subscriber('/pro/voice_detector', String, self.cb_voice_detector)

        self.start_demo = False
        self.is_test = False
        self.cmdline_input = ''
        self.exist_man_5frame = [False] * 5
        self.voice_detector = ''

    def cb_detect_face(self, data):
        """update exist_man_5frame"""
        self.exist_man_5frame.pop(0)
        self.exist_man_5frame.append(data.data)

    def cb_get_cmdline_input(self, data):
        self.cmdline_input = data.data

    def cb_voice_detector(self, data):
        self.voice_detector = data.data

    def main(self):
    # check if man exists
    # if sum(self.exist_man_5frame) >= 3:
    #     self.start_demo = True
    # if self.start_demo is True or self.is_test is True:
        if self.voice_detector == 'Hello':
            # print "success"
            self.start_demo = True
            # firstly after found man 5 times say hello
            self.pub_speak_robot.publish(String('こんにちは。私の名前はターボです。'))
            rospy.sleep(5)
            self.pub_speak_robot.publish(String('あなたの名前はなんですか？'))
            rospy.sleep(10)
            while self.voice_detector == 'Hello':
                self.pub_speak_robot.publish(String('恥ずかしがらないで教えてください'))
                rospy.sleep(10)
            print self.voice_detector
            self.pub_speak_robot.publish(String('%s' % self.voice_detector))
            rospy.sleep(1)
            self.pub_speak_robot.publish(String('さんですか'))
            rospy.sleep(3)
            self.pub_speak_robot.publish(String('いい名前ですね'))
            rospy.sleep(3)
            self.pub_speak_robot.publish(String('何をしますか？'))
            rospy.sleep(9)
            # select
            while not self.voice_detector == ('Photo' or 'Drink' or 'Dance' or 'Play'):
            # while not self.voice_detector == 'Photo':
                self.pub_speak_robot.publish(String('早く決めてください'))
                rospy.sleep(100)
                # cmd = ('gnome-terminal -e "python /home/tomoya/catkin_ws/pro/src/turbo/scripts/get_cmdline_input.py select"')
                # os.system(cmd)
            if self.voice_detector == 'Photo':
                # invite the man to be taken photo
                self.pub_speak_robot.publish(String('写真を撮りましょう！'))
                rospy.sleep(4)
                while all(self.exist_man_5frame) is not True:
                    # wait for the man faces to the camera
                    self.pub_speak_robot.publish(String('カメラに顔を向けてください'))
                    rospy.sleep(10)
                self.pub_speak_robot.publish(String('三・・二・・一'))
                rospy.sleep(4)
                self.pub_speak_robot.publish(String('カシャッ！'))
                rospy.sleep(1)
                self.pub_take_photo.publish()
                rospy.sleep(2)
                self.pub_speak_robot.publish(String('写真を撮りました。・・'
                                                        'メールで送ることができますので、・・'
                                                        'メールアドレスを打ち込んでください。・・'
                                                        'できればGmailでお願いします。'))
                rospy.sleep(13)
                # input email
                cmd = ('gnome-terminal -e "python /home/tomoya/catkin_ws/pro/src/turbo/scripts/get_cmdline_input.py email"')
                os.system(cmd)
                rospy.sleep(10)
                # send mail
                body = ('うまく撮れていますか？\n')
                subject = 'pro_demo.py: Take Photo'
                attachment = '/tmp/pro_take_photo.jpeg'
                cmd = 'echo "{0}" | mutt -s "{1}" -a {2} -- {3}'
                cmd = cmd.format(body, subject, attachment, self.cmdline_input)
                os.system(cmd)
                # say user to check the inbox
                self.pub_speak_robot.publish(String('メールを送りました。・・受信箱を確認してください。'))
                rospy.sleep(7)
            elif self.voice_detector == 'Drink':
                # pass drink
                self.pub_speak_robot.publish(String('どうぞ'))
                rospy.sleep(7)
                self.pub_catch_drink.publish()
                rospy.sleep(25)
            elif self.voice_detector == 'Dance':
                self.pub_speak_robot.publish(String('踊ります'))
                rospy.sleep(5)
                self.pub_dance_robot.publish()
                rospy.sleep(30)
                self.pub_speak_robot.publish(String('意外とうまいでしょ？'))
            elif self.voice_detector == 'Play':
                self.pub_speak_robot.publish(String('何をして遊びますか？'))
                while not self.voice_detector == 'Daruma' or 'Sanpo':
                    self.pub_speak_robot.publish(String('その遊びはわかりません。'))
                if self.voice_detector == 'Daruma':
                    print "だるまさんが転んだ。"
                elif self.voice_detector == 'Sanpo':
                    self.pub_speak_robot.publish(String('散歩をしましょう。'))
                    rospy.sleep(4)
                    self.pub_speak_robot.publish(String('ついていってもいいですか？'))
                    rospy.sleep(7)
                    while not self.voice_detector == 'Yes' or 'No':
                        self.pub_speak_robot.publish(String('はやく答えてください。'))
                        rospy.sleep(7)
                    if self.voice_detector == 'Yes':
                        self.pub_speak_robot.publish(String('では、ついていきます。'))
                        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    elif self.voice_detector == 'No':
                        self.pub_speak_robot.publish(String('では、命令してください'))
                        rospy.sleep(30)
                        
        self.start_demo = False


if __name__ == '__main__':
    pro_demo = ProDemo()
    pro_demo.is_test = False

    rospy.sleep(10)
    while True:
        pro_demo.main()