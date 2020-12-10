#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from ball_detection.msg import PosVelTimed
from robot_controller import RobotController

def callback(msg):
    controller.group.stop()
    controller.move_to_goal(msg.pos.x, msg.pos.y, msg.pos.z)

if __name__ == '__main__':
    rospy.init_node('ball_controller')
    controller = RobotController(5)
    controller.move_to_goal(0, 0, 1)
    sub = rospy.Subscriber('/ball_detection/predicted_ball_state', PosVelTimed, callback, queue_size=10)
    rospy.spin()
