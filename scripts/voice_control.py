#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
 
try:
    import roslib
    roslib.load_manifest('turtlebot_teleop')
    roslib.load_manifest('rospeex_if')
except:
    pass
 
import rospy
from geometry_msgs.msg import Twist
 
from rospeex_if import ROSpeexInterface
import sys, termios, re
 
 
rospeex = None
turtle_pub = None
 
OPERATION_TIME = 100
TARGET_SPEED = 0.2
TARGET_TURN = 0.2
 
_target_speed = 0.0
_control_speed = 0.0
_target_turn = 0.0
_control_turn = 0.0
_operation_count = 0
_start_flag = False
 
def set_operation( target_speed, control_speed, target_turn, control_turn ):
    global _target_speed, _control_speed, _target_turn, _control_turn, _operation_count
    _target_speed = target_speed
    _control_speed = control_speed
    _target_turn = target_turn
    _control_turn = control_turn
    _operation_count = 0
 
def timer_callback(event):
    global _target_speed, _control_speed, _target_turn, _control_turn, _operation_count, OPERATION_TIME, TARGET_SPEED
    _operation_count += 1
    if _operation_count < OPERATION_TIME:
        turtle_move( _target_speed, _control_speed, _target_turn, _control_turn )
    elif _start_flag == True:
        turtle_move( TARGET_SPEED, TARGET_SPEED, 0, 0 )
    else:
        turtle_move( 0, 0, 0, 0 )
 
def sr_response(message):
    global TARGET_SPEED, TARGET_TURN, _start_flag
    print 'you said : %s' %message
    rule = re.compile(u'(?P<operation>(右|左|前|後|スタート|ストップ))')
    uni_msg = unicode(message, 'utf-8')
    m = rule.match(uni_msg)
 
    if m is not None:
        operation = m.group('operation')
        if operation == u"前":
            rospeex.say("前に進みます")
            set_operation(TARGET_SPEED, TARGET_SPEED, 0, 0)
        elif operation == u"後":
            rospeex.say("後に進みます")
            set_operation(-TARGET_SPEED, -TARGET_SPEED, 0, 0)
        elif operation == u"右":
            rospeex.say("右に進みます")
            set_operation(TARGET_SPEED, TARGET_SPEED, -TARGET_TURN, -TARGET_TURN)
        elif operation == u"左":
            rospeex.say("左に進みます")
            set_operation(TARGET_SPEED, TARGET_SPEED, TARGET_TURN, TARGET_TURN)
        elif operation == u"スタート":
            rospeex.say("スタートします")
            _start_flag = True
        elif operation == u"ストップ":
            rospeex.say("ストップします")
            _start_flag = False
 
def turtle_move(target_speed, control_speed, target_turn, control_turn ):
    if target_speed > control_speed:
        control_speed = min( target_speed, control_speed + 0.02 )
    elif target_speed < control_speed:
        control_speed = max( target_speed, control_speed - 0.02 )
    else:
        control_speed = target_speed
 
    if target_turn > control_turn:
        control_turn = min( target_turn, control_turn + 0.1 )
    elif target_turn < control_turn:
        control_turn = max( target_turn, control_turn - 0.1 )
    else:
        control_turn = target_turn
 
    twist = Twist()
    twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
    turtle_pub.publish(twist)
 
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('rospeex_turtle')
    rospeex = ROSpeexInterface()
    rospeex.init()
    rospeex.register_sr_response( sr_response )
    rospeex.set_spi_config(language='ja',engine='nict')
    turtle_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
    rospy.Timer(rospy.Duration(0,100000000), timer_callback)
 
    try:
        rospy.spin();
    except Exception as e:
        print str(e)
    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        turtle_pub.publish(twist)
 
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)