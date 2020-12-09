#! /usr/bin/python

import rospy
import numpy as np
from ball_predict.msg import PosVelTimed
import math
from gazebo_msgs.msg import ModelStates

def get_pose():
	rospy.wait_for_service('gazebo/get_model_state')
   	ball = rospy.ServiceProxy('gazebo/get_model_state', Empty)
	while not rospy.is_shutdown():
		

if __name__ == '__main__':

  rospy.init_node('get_ball_pose_node', anonymous=True)
  
  try:
    get_pose()
  except rospy.ROSInterruptException: pass