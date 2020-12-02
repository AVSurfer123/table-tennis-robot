#!/usr/bin/env python
"""
Path Planning Script for Lab 5
Author: Tiffany Cappellari
"""
import sys

import rospy
import numpy as np
import traceback
import moveit_commander

from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped

group_name = "kr5_planning_group"

def main():
    """
    Main Script
    """

    # Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize the robot
    robot = moveit_commander.RobotCommander()

    # Initialize the planning scene
    scene = moveit_commander.PlanningSceneInterface()

    # This publishes updates to the planning scene
    planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

    # Instantiate a move group
    group = moveit_commander.MoveGroupCommander(group_name)

    # Set the maximum time MoveIt will try to plan before giving up
    group.set_planning_time(5)

    # Set maximum velocity scaling
    group.set_max_velocity_scaling_factor(1.0)
    

	# K values for Part 5
    Kp = 0.1 * np.array([0.3, 2, 1, 1.5, 2, 2, 3]) # Borrowed from 106B Students
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.5, 0.5, 0.5]) # Borrowed from 106B Students
    Ki = 0.01 * np.array([1, 1, 1, 1, 1, 1, 1]) # Untuned
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9]) # Untuned

	# Initialize the controller for Part 5
	# controller = Controller( . . . )

    #-----------------------------------------------------#
    ## Add any obstacles to the planning scene here

    #-----------------------------------------------------#

    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;

    def move_to_goal(x, y, z, or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0, orien_const=[]):
        while not rospy.is_shutdown():
            try:
                goal = PoseStamped()
                goal.header.frame_id = "world"

                #x, y, and z position
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = z

                #Orientation as a quaternion
                goal.pose.orientation.x = or_x
                goal.pose.orientation.y = or_y
                goal.pose.orientation.z = or_z
                goal.pose.orientation.w = or_w

                group.set_pose_target(goal)
                group.set_start_state_to_current_state()

                constraints = Constraints()
                constraints.orientation_constraints = orien_const
                group.set_path_constraints(constraints)

                plan = group.plan()

                raw_input("Press <Enter> to move the right arm to goal pose: ")

                # Might have to edit this for part 5
                if not group.execute(plan, True):
                    print("Execution failed")

            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break

    # Set your goal positions here
    move_to_goal(0.47, 0.5, 0.5)

        

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
