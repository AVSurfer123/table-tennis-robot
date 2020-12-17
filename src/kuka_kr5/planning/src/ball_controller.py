#!/usr/bin/env python

from __future__ import print_function

import sys
import math
from collections import deque

import rospy
from ball_detection.msg import PosVelTimed
from robot_controller import RobotController

HOME_POSE = [0, 0, 1]
HOME_ORI = [-0.00060612617037, 0.98890581114, 0.148542219849, 0.000371788566274]
HOME = HOME_POSE + HOME_ORI

moving = False

def callback(msg):
    global moving
    if msg.hittable:
        moving = True
        goal = [msg.pos.x, msg.pos.y, msg.pos.z]
        print('Moving arm to:', goal)
        # controller.hit_ball(goal, HOME_ORI)
        # time = rospy.Time.now()
        controller.move_to_goal(*(goal + HOME_ORI), time=msg.header.stamp)
        # goal[1] -= .2
        # total_nsecs = time.nsecs + 1e8
        # fractional, integer = math.modf(total_nsecs/1e9)
        # time.secs += int(integer)
        # time.nsecs += fractional*1e9
        # controller.move_to_goal(*(goal + HOME_ORI), time=time)
    elif moving:
        moving = False
        controller.move_to_goal(*HOME)
    # elif moving:
    #     moving = False
    #     controller.move_to_goal(*HOME)

if __name__ == '__main__':
    rospy.init_node('ball_controller')
    controller = RobotController(8)
    controller.move_to_goal(*HOME)
    sub = rospy.Subscriber('/ball_detection/predicted_ball_state', PosVelTimed, callback, queue_size=10)
    rospy.spin()
