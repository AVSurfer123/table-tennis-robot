#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import PoseStamped, Pose, Point
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import moveit_commander

rospy.init_node('add_collision')

# Initialize moveit_commander
moveit_commander.roscpp_initialize(sys.argv)

# Initialize the robot
robot = moveit_commander.RobotCommander()

# Initialize the planning scene
scene = moveit_commander.PlanningSceneInterface()

pub = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

TABLE_CENTER = PoseStamped()
TABLE_CENTER.header.frame_id = 'world'
TABLE_CENTER.pose = Pose(position=Point(0, -1.37, .76))
TABLE_CENTER.pose.orientation.w = 1

TABLE_SIZE = [1.525, 2.74, 0.0201]

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

print(co)
count = 0

while not rospy.is_shutdown():
    pub.publish(co)
    co.primitive_poses[0].position.x += 1
    count += 1
    print(count)
    rospy.sleep(0.5)

