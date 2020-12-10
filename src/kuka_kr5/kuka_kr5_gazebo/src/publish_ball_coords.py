#!/usr/bin/env python

import rospy

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped

BALL_NAME = 'ping_pong_ball'

def callback(msg):
    for i, model in enumerate(msg.name):
        if model == BALL_NAME:
            pose = PoseStamped()
            pose.pose = msg.pose[i]
            pose.header.frame_id = 'world'
            pose.header.stamp = rospy.Time.now()
            pub.publish(pose)


if __name__ == '__main__':
    rospy.init_node('true_ball_pose')
    sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback, queue_size=10)
    pub = rospy.Publisher('/ball_detection/true_pose', PoseStamped, queue_size=10)
    # while not rospy.is_shutdown():
    rospy.spin()