#!/usr/bin/env python

from __future__ import print_function
import sys
from collections import deque
import copy

import numpy as np
import rospy
from ros_numpy import numpify
from geometry_msgs.msg import PointStamped
from ball_detection.msg import PosVelTimed


X_VAR = np.array([
    [.04, 0],
    [0, .04]
])

Y_VAR = np.array(.4)

def kalman(pos, time, direction):
    if direction == 'x' or direction == 'y':
        acc = 0
    elif direction == 'z':
        acc = -9.81
    else:
        raise ValueError("direction is not one of x, y, z")

    x = np.array([pos[0], (pos[1] - pos[0])/(time[1] - time[0])])
    print("Start:", x)
    sigma = np.array([
        [.04, 0],
        [0, .4]
    ])

    for i in range(1, len(time)):
        t = time[i] - time[i-1]
        A = np.array([
            [1, t],
            [0, 1]
        ])
        B = np.array([
            [.5 * acc * t * t],
            [acc * t]
        ])
        C = np.array([[1., 0]])

        # Predict 1 timestep into future
        x = A.dot(x) + B
        sigma = A.dot(sigma).dot(A.T) + X_VAR

        # Calculate gain
        K = sigma.dot(C.T).dot(np.linalg.inv(C.dot(sigma).dot(C.T) + Y_VAR))
        
        # Update current estimates of state and variance using observation
        x = x + K.dot(pos[i] - C.dot(x))
        sigma = (np.eye(len(x)) - K.dot(C)).dot(sigma)

    return x, sigma

MIXING = [0.25, 0.25, 0.5]  # x, y, z

def ema(pos, time, direction, mix_constant):
    acc = 0
    if direction == 'x' or direction == 'y':
        acc = 0
    elif direction == 'z':
        acc = -9.81
    pos_filtered = np.array(pos[0:2])
    vel_filtered = np.array([(pos[1]-pos[0]) / (time[1]-time[0])])
    count_z = 0
    for i in range(2,len(time)):
        dt = time[i]-time[i-1]
        curVel = (pos[i]-pos[i-1]) / dt
        if direction == 'z' and curVel > 0 and count_z < 3:
            vel_filtered = np.append(vel_filtered,curVel)
            pos_filtered = np.append(pos_filtered,pos[i])
            count_z = count_z + 1
            continue
        predVel = vel_filtered[-1] + acc * dt
        estVel = (1-mix_constant) * predVel + mix_constant * curVel
        vel_filtered = np.append(vel_filtered,estVel)
        
        predPos = pos_filtered[-1] + vel_filtered[-2] * dt + 0.5 * acc * dt**2
        estPos = (1-mix_constant) * predPos + mix_constant * pos[i]
        pos_filtered = np.append(pos_filtered,estPos)
                             
    return pos_filtered, vel_filtered

last_msg = None
history = deque([], maxlen=15)

def convert_to_numpy(history):
    pos = []
    times = []
    for msg in history:
        times.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
        pos.append([msg.point.x, msg.point.y, msg.point.z])
    return np.array(pos).T, np.array(times)

def vel_callback(pose_msg):
    global history, last_msg

    first = False
    if last_msg is None:
        first = True
    elif pose_msg.header.stamp.secs - last_msg.header.stamp.secs > 1:
        first = True
    
    if first:
        history = [pose_msg]
    else:
        history.append(pose_msg)
    
    # Can only determine velocity with 2 points
    if len(history) < 2:
        last_msg = pose_msg
        return

    pos, time = convert_to_numpy(history)

    filtered = PosVelTimed()
    filtered.header.frame_id = 'world'
    filtered.header.stamp = pose_msg.header.stamp

    kalman_msg = copy.deepcopy(filtered)

    for i, (direction, mix) in enumerate(zip(['x', 'y', 'z'], MIXING)):
        pos_filtered, vel_filtered = ema(pos[i], time, direction, mix)
        setattr(filtered.pos, direction, pos_filtered[-1])
        setattr(filtered.vel, direction, vel_filtered[-1])

        state_kalman, _ = kalman(pos[i], time, direction)
        setattr(kalman_msg.pos, direction, state_kalman[0, 0])
        setattr(kalman_msg.vel, direction, state_kalman[0, 1])
        print("Position delta:", pos_filtered[-1] - state_kalman[0, 0])
        print("Velocity delta:", vel_filtered[-1] - state_kalman[0, 1])

    vel_pub.publish(filtered)
    kalman_pub.publish(kalman_msg)

    filtered = kalman_msg
    # print('Filtered position:', filtered.pos.x, filtered.pos.y, filtered.pos.z)
    # print("Filtered velocity:", filtered.vel.x, filtered.vel.y, filtered.vel.z)
    
    last_msg = pose_msg


if __name__ == '__main__':
    rospy.init_node('velocity_filter')
    pose_sub = rospy.Subscriber('/ball_detection/ball_pose', PointStamped, vel_callback, queue_size=10)
    vel_pub = rospy.Publisher('/ball_detection/ball_ema_filtered', PosVelTimed, queue_size=10)
    kalman_pub = rospy.Publisher('/ball_detection/ball_kalman_filtered', PosVelTimed, queue_size=10)
    rospy.spin()

