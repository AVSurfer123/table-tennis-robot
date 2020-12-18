#!/usr/bin/env python

import sys
import math
import numpy as np
from scipy.optimize import fsolve
import rospy
from ball_detection.msg import PosVelTimed


g = -9.81
e = 0.87
# set target point
x_t = 0
y_t = -2.055
z_t = 0.78
sin = math.sin
cos = math.cos
vy_out=3.0

def yaw_vp_eq(var,x,y,z,vx,vy,vz):
    yaw = var[0]
    vp = var[1]

    # time for the flight
    t = (y_t - y) / vy_out
    vx_out = (x_t - x) / t

    eq1 = vx*(1-e*yaw**2) - vy*(1+e)*yaw - (1+e)*vp*yaw - vx_out
    eq2 = -vx*(1+e)*yaw + vy*(yaw**2 - e) - (1+e)*vp - vy_out

    return np.array([eq1, eq2])

def roll_vp_eq(var,x,y,z,vx,vy,vz):
    roll = var[0]
    vp = var[1]

    t = (y_t - y) / vy_out
    vz_out = (z_t - t) / t - 0.5 * g * t

    eq1 = vy*(roll**2 - e) - vz*roll*(1+e) - (1+e)*vp - vy_out
    eq2 = -vy*roll*(1+e) + vz*(1 - e*roll**2) - (1+e)*vp*roll - vz_out

    return np.array([eq1, eq2])


def euler_to_quaternion(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qw, qx, qy, qz]

# The method below will calculate the yaw and roll angle of the paddle, the pitch angle is not important
#   Angles are with respect to the world frame
#   It also calculate the velocity of paddle vp w/r to the world frame
def angle(x,y,z,vx,vy,vz):
    yaw_vp0 = np.array([0,0])
    yaw_vp = fsolve(yaw_vp_eq, yaw_vp0, args=(x,y,z,vx,vy,vz))
    print("yaw_vp = ", yaw_vp)

    roll_vp0 = np.array([0,0])
    roll_vp = fsolve(roll_vp_eq, roll_vp0, args=(x,y,z,vx,vy,vz))
    print("roll_vp = ", roll_vp)

    print(" ")

    vp = (yaw_vp[1]+roll_vp[1])/2
    angle1 = yaw_vp[0]+3.14
    angle3 = roll_vp[0]-1.57

    return euler_to_quaternion(angle1,0,angle3), np.array([vp*sin(angle1),-vp*cos(angle1),vp*sin(angle3)])


def callback(msg):
    global x,y,z,vx,vy,vz
    x = msg.pos.x
    y = msg.pos.y
    z = msg.pos.z
    vx = msg.vel.x
    vy = msg.vel.y
    vz = msg.vel.z

    if vy == 0 or vz == 0 or vy < 0:
        return

    angle(x,y,z,vx,vy,vy)


if __name__ == '__main__':
    rospy.init_node('paddle_angle_calc')
    sub = rospy.Subscriber('/ball_detection/predicted_ball_state', PosVelTimed, callback, queue_size=10)
    rospy.spin()