#!/usr/bin/env python

import sys
import math

import rospy
from ball_detection.msg import PosVelTimed



def quadratic(a,b,c):
    d = b**2 - 4*a*c
    if d >= 0:
        return (-b + math.sqrt(d)) / (2*a), (-b + math.sqrt(d)) / (2*a)
    else:
        return None, None



# The method below will calculate the yaw and roll angle of the paddle, the pitch angle is not important
#   Angles are with respect to the world frame
#   It also calculate the velocity of paddle vp w/r to the world frame
def angle(x,y,z,vx,vy,vz,vy_out=3.0):
    # arguments are the end state of the incoming ball, set defualt hit back y vel to 3.0 m/s

    g = -9.81
    e = 0.87
    # set target point
    x_t = 0
    y_t = -2.055
    z_t = 0.78

    if vy == 0 or vz == 0 or vy < 0:
        return

    # time for the flight
    t = (y_t - y) / vy_out
    vx_out = (x_t - x) / t


    yaw, roll,  = 0.0, 0.0

    # Calculate the yaw angle from vx and vy
    tolerance = 0.01
    if abs(vx) < tolerance:
        yaw = vx_out / (vy*(1-e))
    else:
        yaw1, yaw2 = quadratic(vx*e, vy*(1-e), vx-vx_out)
        if yaw1 is None and yaw2 is None:
            print("cannot find a solution for the yaw angle, set to zero.")
        else:
            if yaw1 is not None and yaw2 is None:
                yaw = yaw1
            elif yaw1 is None and yaw2 is not None:
                yaw = yaw2
            # if both solutions are valid
            else:
                # use the x position to determine an appropriate angle
                if x >= 0:
                    if yaw1 >=0 and yaw2 < 0:
                        yaw = yaw1
                    elif yaw1 < 0 and yaw2 >= 0:
                        yaw = yaw2
                    else:
                        yaw = min(abs(yaw1), abs(yaw2))
                else:
                    if yaw1 >=0 and yaw2 < 0:
                        yaw = yaw2
                    elif yaw1 < 0 and yaw2 >= 0:
                        yaw = yaw1
                    else:
                        yaw = -min(abs(yaw1), abs(yaw2))

    # Calculate the x and y component of vp
    vp = vx * (1-e)/e *yaw - vy/e * yaw**2 - vy + vy_out/e
    vpx = -vp * math.sin(yaw)
    vpy = -vp * math.cos(yaw)


    # Calculate the roll and gle from vy and vz
    if abs(vz) < tolerance:
        roll = (vz - vz_out) / (vy*(1+e))
    vz_out = (z_t - t) / t - 0.5 * g * t
    roll1, roll2 = quadratic(vz*e, vy*(1+e), vz_out-vz)
    if roll1 is None and roll2 is None:
        print("cannot find a solution for the roll angle, set to zero.")
    else:
        if roll1 is not None and roll2 is None:
            roll = roll1
        elif roll1 is None and roll2 is not None:
            roll = roll2
        # if both solutions are valid
        else:
            roll = min(abs(roll1), abs(roll2))

    # Calculate the z component of vp
    vpz = -vp*math.sin(roll)

    # Calculate the difference of vpy
    vpy_r = -vp*math.cos(roll)
    print("error of vpy: ", abs(vpy - vpy_r))

    print("Results: ")
    print("angles[yaw, roll]: ", yaw, roll)
    print("paddle vel: ", vpx, vpy, vpz)
    print(" ")

    return [yaw,roll], [vpx, vpy, vpz]

def callback(msg):
    x = msg.pos.x
    y = msg.pos.y
    z = msg.pos.z
    vx = msg.vel.x
    vy = msg.vel.y
    vz = msg.vel.z
    angle(x,y,z,vx,vy,vy)


if __name__ == '__main__':
    rospy.init_node('paddle_angle_calc')
    sub = rospy.Subscriber('/ball_detection/predicted_ball_state', PosVelTimed, callback, queue_size=10)
    rospy.spin()