#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = ['joint%d' % (i+1) for i in range(6)]

if __name__ == '__main__':
    rospy.init_node('arm_test')
    client = actionlib.SimpleActionClient('/x6/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    goal = FollowJointTrajectoryGoal()
    print(JOINT_NAMES)
    goal.trajectory.joint_names = JOINT_NAMES
    goal_pos = [np.pi/6, np.pi/6, np.pi/2, 0, 0, 0]
    goal_pos = [0] * 6
    goal.trajectory.points.append(JointTrajectoryPoint(positions=goal_pos, time_from_start=rospy.Duration(.2)))

    client.send_goal(goal)
    client.wait_for_result()
