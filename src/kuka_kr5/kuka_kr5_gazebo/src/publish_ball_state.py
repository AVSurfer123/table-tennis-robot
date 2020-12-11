#!/usr/bin/env python

import rospy
import copy

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from ball_detection.msg import PosVelTimed

BALL_NAME = 'ping_pong_ball'

latest_msg = None

def callback(msg):
    global latest_msg
    latest_msg = msg

def publish(event):
    msg = copy.deepcopy(latest_msg)
    if msg == None:
        return
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
    frequency = 60
    rospy.Timer(rospy.Duration(1.0/frequency), publish)
    rospy.spin()
