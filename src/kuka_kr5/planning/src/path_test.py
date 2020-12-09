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

from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, Point
from shape_msgs.msg import SolidPrimitive

group_name = "kr5_planning_group"

TABLE_CENTER = PoseStamped()
TABLE_CENTER.header.frame_id = 'world'
TABLE_CENTER.pose = Pose(position=Point(0, -1.37, .76))
TABLE_CENTER.pose.orientation.w = 1

TABLE_SIZE = [1.525, 2.74, 0.0201]


def move_to_goal(x, y, z, or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0, orien_const=[]):
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

        traj = group.plan()

        new_traj = RobotTrajectory()
        new_traj.joint_trajectory.header = traj.joint_trajectory.header
        new_traj.joint_trajectory.joint_names = traj.joint_trajectory.joint_names
        n_joints = len(traj.joint_trajectory.joint_names)
        n_points = len(traj.joint_trajectory.points)
        spd = 5.0
        print(traj.joint_trajectory.points)

        for i in range(n_points):
            new_traj.joint_trajectory.points.append(JointTrajectoryPoint())
            new_traj.joint_trajectory.points[i].time_from_start = traj.joint_trajectory.points[i].time_from_start / spd
            if len(traj.joint_trajectory.points[i].velocities) != n_joints:
                print(traj.joint_trajectory.points[i].velocities)
            for j in range(len(traj.joint_trajectory.points[i].velocities)):
                new_traj.joint_trajectory.points[i].velocities.append(traj.joint_trajectory.points[i].velocities[j] * spd)
                new_traj.joint_trajectory.points[i].accelerations.append(traj.joint_trajectory.points[i].accelerations[j] * spd * spd)
                new_traj.joint_trajectory.points[i].positions.append(traj.joint_trajectory.points[i].positions[j])

        # _ = raw_input("Press <Enter> to move the arm to goal pose: ")

        if not group.execute(new_traj, True):
            print("Execution failed")

    except Exception as e:
        traceback.print_exc()

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
    scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

    # Instantiate a move group
    global group
    group = moveit_commander.MoveGroupCommander(group_name)

    # Set the maximum time MoveIt will try to plan before giving up
    group.set_planning_time(5)

    # Set maximum velocity scaling
    group.set_max_velocity_scaling_factor(1.0)
    group.set_max_acceleration_scaling_factor(1.0)

    print(group.get_current_pose())
    print(group.get_end_effector_link())
    print(group.get_pose_reference_frame())
    print(group.get_goal_tolerance())

    # Create a CollisionObject, which will be added to the planning scene
    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = 'table'
    co.header = TABLE_CENTER.header

    # Create a box primitive, which will be inside the CollisionObject
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = TABLE_SIZE

    # Fill the collision object with primitive(s)
    co.primitives = [box]
    co.primitive_poses = [TABLE_CENTER.pose]

    # Publish the object (don't know why but need to publish twice)
    scene_publisher.publish(co)
    rospy.sleep(0.5)
    scene_publisher.publish(co)
    rospy.sleep(0.5)

    # group.set_goal_joint_tolerance(.01)
    # group.set_goal_position_tolerance(.01)
    # group.set_goal_orientation_tolerance(.001)

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

    # Set your goal positions here
    # move_to_goal(0, 0, 1)
    if len(sys.argv) > 3:
        move_to_goal(*(float(num) for num in sys.argv[1:4]))
    else:
        move_to_goal(0.5, .3, 0.76)
    # rospy.sleep(1)
    # move_to_goal(.6, 1, 1)

        

if __name__ == '__main__':
    rospy.init_node('path_planning')
    main()
