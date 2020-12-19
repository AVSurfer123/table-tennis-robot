#!/usr/bin/env python3.7

import time
import numpy as np
import zmq
import pyjoycon

# compensate the accelerometer readings from gravity. 
# @param q the quaternion representing the orientation of a 9DOM MARG sensor array
# @param acc the readings coming from an accelerometer expressed in g
#
# @return a 3d vector representing dinamic acceleration expressed in g
def gravity_compensate(q, acc):
  g = [0.0, 0.0, 0.0]
  
  # get expected direction of gravity
  g[0] = 2 * (q[1] * q[3] - q[0] * q[2])
  g[1] = 2 * (q[0] * q[1] + q[2] * q[3])
  g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
  
  # compensate accelerometer readings with the expected direction of gravity
  return [acc[0] - g[0], acc[1] - g[1], acc[2] - g[2]]

if __name__ == '__main__':

    joy_id = pyjoycon.get_R_id()
    joycon = pyjoycon.GyroTrackingJoyCon(*joy_id)
    
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    PORT = 6000
    socket.bind("tcp://127.0.0.1:%s" % PORT)

    while True:
        accel_x = joycon.get_accel_x()
        accel_y = joycon.get_accel_y()
        accel_z = joycon.get_accel_z()
        accel = np.array(joycon.accel_in_g).mean(axis=0) * 9.81
        gyro = np.array(joycon.gyro_in_deg).mean(axis=0)
        quat = joycon.direction_Q
        linear_acc = gravity_compensate(quat, accel)
        print(quat)
        print(accel)
        print(linear_acc)
        print(joycon.direction)
        print(joycon.rotation)
        print(joycon.pointer)
        time.sleep(.1)
