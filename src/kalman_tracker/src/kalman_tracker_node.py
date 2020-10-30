#!/usr/bin/env python
import rospy
from arm_moveit_trajectory.msg import ball_state
from natnet_node.msg import unid_markers
from python_tracker import LinearKF
from ekf_tracker.EKF_tracker import EkfTracker
#from kalman_tracker import LinearKF
import numpy as np
import tf
from tf.transformations import quaternion_from_euler
from rospy.numpy_msg import numpy_msg

pub = None
#KF = LinearKF(1.0 / 120)
TIMESTEP = 1.0/120
KF = EkfTracker(TIMESTEP)
last_sample_time = None
R_to_ref = None  # Matriz de transformacao homogenea (4x4) para o referencial do biorob
biorob_loc = np.zeros(3)  # localizacao da base do biorob

VECTOR4_BUFFER = np.ones((4, 1))

def filter_markers_too_close(markers):
    """
    Returns list with only the markers outside a 25cm radius of the location of the robot base (to exclude the base
    markers)
    :param markers: list of markers
    :return: list of markers outside radius
    """
    global biorob_loc
    gen_bior_mk = (convert_to_bior_ref(m.x, m.y, m.z) for m in markers)
    #return [m for m in gen_bior_mk if m[2] > 0.0 and np.linalg.norm(m) > 0.25]
    #return [m for m in gen_bior_mk if m[2] < -0.15 and np.linalg.norm(m) > 0.25]
    return [m for m in gen_bior_mk if np.linalg.norm(m) > 0.25]
    #return [m for m in markers if m.y > 0.0 and np.linalg.norm(np.array([m.x, m.y, m.z]) - biorob_loc) > 0.25]


def convert_to_bior_ref(x, y, z, out=np.ones((4, 1))):
    global R_to_ref
    out[:3, 0] = (x, y, z)
    out = R_to_ref.dot(out)
    return out[:3, 0]


def inverse_homogeneous_transform(H):
    H[:3, 3] = - H[:3, :3].transpose().dot(H[:3, 3])
    H[:3, :3] = H[:3, :3].transpose()

def choose_marker(unid_markers, kf_prediction):

    """

:rtype : object
"""
    if len(unid_markers) > 1:
        minind, mindif = 0, 100000000
        mrk_np = np.zeros((3, 1))
        for ind, marker in enumerate(unid_markers):
            mrk_np=marker#[:3, 0] = convert_to_bior_ref(marker.x, marker.y, marker.z, VECTOR4_BUFFER)
            #print(marker)
            #print(kf_prediction)
            dif = np.linalg.norm(mrk_np - kf_prediction[:3])
            if dif < mindif:
                minind, mindif = ind, dif
        return unid_markers[minind]
    elif len(unid_markers) == 1:
        return unid_markers[0]
    else:
        return 0

#@profile
def callback_unid(data):
    global pub
    global last_sample_time
    global R_to_ref

    bs = ball_state()

    #TODO: Escolher marker mais provavel caso haja mais do que um usando a previsao do KF
    dt = data.stamp - last_sample_time
    #markers_ls = filter_markers_too_close(data.unid_markers)
    markers_np = filter_markers_too_close(data.unid_markers)
    if len(markers_np) == 1:
        marker = markers_np[0]
    elif len(markers_np) > 1:
        marker = choose_marker(markers_np, KF.get_state(dt.to_sec()))
    else:
        marker = None

    if marker is not None and R_to_ref is not None:
        last_sample_time = data.stamp  # actualizar para que o proximo dt esteja correcto

        mhl_dist = KF.update(
            marker.transpose(),
            dt.to_sec()) # TIMESTEP) # Simulation has High Jitter
        bs.point.x, bs.point.y, bs.point.z, bs.vel.x, bs.vel.y, bs.vel.z, bs.acc.x, bs.acc.y, bs.acc.z = KF.get_state()

        if mhl_dist:
            bs.mhl_dist = mhl_dist
        else:
            bs.mhl_dist = EkfTracker.MHD_THRESHOLD ** 2

        bs.stamp = data.stamp

        pub.publish(bs)


def tracker_node():
    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    global R_to_ref
    global biorob_loc

    rospy.init_node('sphere_tracker_node', anonymous=True)

    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('optitrack_frame', 'world', rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException):
            continue

    if rospy.is_shutdown():
        return

    tros = tf.TransformerROS()
    R_to_ref = tros.fromTranslationRotation(trans, rot)

    #(0,0,0) converter (0,0,0) do frame do biorob para o do optitrack <=> posicao do biorob no frame do optitrack
    H = np.copy(R_to_ref)
    inverse_homogeneous_transform(H)
    biorob_loc = np.ones((4, 1))
    biorob_loc[:3, 0] = (0, 0, 0)
    biorob_loc = H.dot(biorob_loc)[:3, 0]

    #rospy.Subscriber("optitrack_markers", optitrack, callback)
    rospy.Subscriber("unid_markers", unid_markers, callback_unid)

    #Mudar para a mensagem do estado da bola
    global pub
    global last_sample_time
    last_sample_time = rospy.get_rostime()
    pub = rospy.Publisher('ball_state', ball_state, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    tracker_node()
