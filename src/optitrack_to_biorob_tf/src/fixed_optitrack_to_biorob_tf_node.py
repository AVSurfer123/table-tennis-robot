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

    #o = msg.rigid_bodies[0].orientation
    #p = msg.rigid_bodies[0].position
    #pos = (p.x, p.y, p.z)
    #ori = (o.x, o.y, o.z, o.w)
    br.sendTransform(pos,
                     ori,
                     rospy.Time.now(),
                     "optitrack_frame",
                     "world")

if __name__ == '__main__':
    rospy.init_node('optitrack_to_biorob_tf_publisher')
    #rospy.Subscriber('/rigid_bodies',
    #                 rigid_bodies,
    #                 handle_biorob_pose)
    #H=np.array([[0.99994617, -0.00259749, -0.01004561, 0.2294877],
    #            [0.01008376, 0.0151163, 0.99983489, 0.02752423],
    #            [-0.00244521, -0.99988237, 0.01514168, -1.616207],
    #            [0.0,        0.0,          0.0,          1.0]])

   #rosrun ttf_echo optitrack_frame world
   #At time 1451498029.310
   #- Translation: [0.332, -1.582, 0.015]
   #- Rotation: in Quaternion [-0.705, -0.021, 0.023, 0.709]
   #        in RPY (radian) [-1.565, 0.002, 0.062]
   #        in RPY (degree) [-89.649, 0.121, 3.558]



    #rosrun tf tf_echo world optitrack_frame
    #At time 1451498175.534
    #- Translation: [-0.233, 0.026, 1.599]
    #- Rotation: in Quaternion [0.705, 0.021, -0.023, 0.709]
    #        in RPY (radian) [1.565, 0.062, -0.002]
    #        in RPY (degree) [89.654, 3.549, -0.133]


    pos = (0.332, -1.582, 0.015)
    ori = (-0.705, -0.021, 0.023, 0.709)
    while(True):
        br = tf.TransformBroadcaster()
            #pos = (p.x, p.y, p.z)
            #ori = (o.x, o.y, o.z, o.w)
        br.sendTransform(pos,
                             ori,
                             rospy.Time.now(),
                             "optitrack_frame",
                             "world")
        rospy.sleep(0.1)
    rospy.spin()
