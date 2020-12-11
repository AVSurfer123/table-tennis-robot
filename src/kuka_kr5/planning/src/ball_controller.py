#!/usr/bin/env python

from __future__ import print_function

import sys
from collections import deque

import rospy
from ball_detection.msg import PosVelTimed
from robot_controller import RobotController

HOME_POSE = (0, 0, 1)
HOME_ORI = (-0.00060612617037, 0.98890581114, 0.148542219849, 0.000371788566274)
HOME = HOME_POSE + HOME_ORI

moving = False

def callback(msg):
    global moving
    if msg.hittable:
        moving = True
        goal = msg.pos.x, msg.pos.y, msg.pos.z
        print('Moving arm to:', goal)
        controller.move_to_goal(*(goal + HOME_ORI))
        rospy.sleep(1)
        controller.move_to_goal(*HOME)
    # elif moving:
    #     moving = False
    #     controller.move_to_goal(*HOME)

if __name__ == '__main__':
    rospy.init_node('ball_controller')
    controller = RobotController(10)
    controller.move_to_goal(*HOME)
    sub = rospy.Subscriber('/ball_detection/predicted_ball_state', PosVelTimed, callback, queue_size=10)
    rospy.spin()
