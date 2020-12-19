#!/usr/bin/env python

from __future__ import print_function

import sys
import math
from collections import deque
import numpy as np

import rospy
from ball_detection.msg import PosVelTimed
from robot_controller import RobotController
from paddle_angle_dummy import angle

HOME_POSE = [0, 0, 1]
HOME_ORI = [-0.00060612617037, 0.98890581114, 0.148542219849, 0.000371788566274]
HOME = HOME_POSE + HOME_ORI

moving = False
count = 0
hit = False

def callback(msg):
    global moving, count, hit

    if msg.vel.y > 0 and not moving:
        hit = False
    if not msg.hittable:
        count = 0
    else:
        count = count + 1

    if msg.hittable and count == 2 and not hit:
        moving = True
        hit = True

        goal = [msg.pos.x, msg.pos.y, msg.pos.z]
        print('Moving arm to:', goal)
        # controller.hit_ball(goal, HOME_ORI)
        # time = rospy.Time.now()
        ori,euler = angle(msg.pos.x, msg.pos.y, msg.pos.z, msg.vel.x, msg.vel.y, msg.vel.z)
        controller.move_to_goal(*(goal + ori))

        # time = rospy.Time.now()
        # curTime = time.secs + time.nsecs/1e9
        # predTime = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        # adv = 0.15*np.linalg.norm(np.array(goal)-np.array(HOME_POSE))
        # print("adv: ", adv)
        # while curTime <= predTime - adv:
        #     time = rospy.Time.now()
        #     curTime = time.secs + time.nsecs/1e9
        #     predTime = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        
        # distance from the ball predction point to the end of hit back trajectory
        dist = 0.25

        roll = euler[0]
        yaw = euler[2]
        sin = math.sin
        cos = math.cos
        goal = [msg.pos.x+dist*sin(yaw)*cos(roll), msg.pos.y-dist*cos(yaw)*cos(roll), msg.pos.z-dist*sin(roll)]
        print("goal_hit:", goal)
        controller_hit.move_to_goal(*(goal + ori), time=msg.header.stamp+rospy.Duration(0.1))
        # goal[1] -= .2
        # total_nsecs = time.nsecs + 1e8
        # fractional, integer = math.modf(total_nsecs/1e9)
        # time.secs += int(integer)
        # time.nsecs += fractional*1e9
        # controller.move_to_goal(*(goal + HOME_ORI), time=time)
    elif moving and hit:
        moving = False
        hit = False
        controller.move_to_goal(*HOME)
    # elif moving:
    #     moving = False
    #     controller.move_to_goal(*HOME)

if __name__ == '__main__':
    rospy.init_node('ball_controller')
    controller = RobotController(8)

    # hit back controller
    controller_hit = RobotController(8)


    controller.move_to_goal(*HOME)
    sub = rospy.Subscriber('/ball_detection/predicted_ball_state', PosVelTimed, callback, queue_size=10)
    rospy.spin()
