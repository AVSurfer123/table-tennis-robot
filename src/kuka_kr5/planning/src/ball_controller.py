#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from ball_detection.msg import PosVelTimed
from robot_controller import RobotController

NATRUAL = True
HIT = False

def callback(msg):
    global NATRUAL, HIT
    if not msg.hittable:
        if HIT:
            controller.move_to_goal(0, 0, 1)
            NATRUAL = True
            HIT = False
            rospy.sleep(1)
            return
        else:
            return
    controller.group.stop()
    controller.move_to_goal(msg.pos.x, msg.pos.y, msg.pos.z)
    NATRUAL = False
    HIT = True
    rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('ball_controller')
    controller = RobotController(6)
    controller.move_to_goal(0, 0, 1)
    sub = rospy.Subscriber('/ball_detection/predicted_ball_state', PosVelTimed, callback)
    rospy.spin()
