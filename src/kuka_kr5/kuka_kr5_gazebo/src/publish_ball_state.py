#!/usr/bin/env python

import rospy

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from ball_detection.msg import PosVelTimed

BALL_NAME = 'ping_pong_ball'

def callback(msg):
    for i, model in enumerate(msg.name):
        if model == BALL_NAME:
            state = PosVelTimed()
            state.pos = msg.pose[i].position
            state.vel = msg.twist[i].linear           
            state.header.frame_id = 'world'
            state.header.stamp = rospy.Time.now()
            pub.publish(state)


if __name__ == '__main__':
    rospy.init_node('true_ball_pose')
    sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback, queue_size=10)
    pub = rospy.Publisher('/ball_detection/ground_truth', PosVelTimed, queue_size=10)
    rospy.spin()