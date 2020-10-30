#!/usr/bin/env python
import roslib
roslib.load_manifest('optitrack_to_biorob_tf')
import rospy

import tf
from natnet_node.msg import rigid_bodies

R_to_ref = None


def inverse_homogeneous_transform(H):
    H[:3, 3] = - H[:3, :3].transpose().dot(H[:3, 3])
    H[:3, :3] = H[:3, :3].transpose()


def handle_biorob_pose(msg):
    global R_to_ref

    br = tf.TransformBroadcaster()
#    tros = tf.TransformerROS()

#    if data.rigid_bodies and R_to_ref is None:  #only run once
#        #expects tuples
#        o = data.rigid_bodies[0].orientation
#        p = data.rigid_bodies[0].position
#        pos = (p.x, p.y, p.z)
#        ori = (o.x, o.y, o.z, o.w)
#        R_to_ref = tros.fromTranslationRotation(pos, ori)
#        inverse_homogeneous_transform(R_to_ref)

    o = msg.rigid_bodies[0].orientation
    p = msg.rigid_bodies[0].position
    pos = (p.x, p.y, p.z)
    ori = (o.x, o.y, o.z, o.w)
    br.sendTransform(pos,
                     ori,
                     rospy.Time.now(),
                     "optitrack_frame",
                     "world")

if __name__ == '__main__':
    rospy.init_node('optitrack_to_biorob_tf_publisher')
    rospy.Subscriber('/rigid_bodies',
                     rigid_bodies,
                     handle_biorob_pose)
    rospy.spin()
