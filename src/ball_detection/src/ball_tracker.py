#! /usr/bin/python

from __future__ import print_function
import sys

import cv2
import message_filters
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from ball_detection.msg import PosVelTimed
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import numpy as np
import tf2_ros as tf2
from tf import transformations
from ros_numpy import numpify

top_img_pub = rospy.Publisher('/ball_detection/top_camera/filtered_image', Image, queue_size=10)
side_img_pub = rospy.Publisher('/ball_detection/side_camera/filtered_image', Image, queue_size=10)
ball_pose_pub = rospy.Publisher('/ball_detection/ball_pose', PointStamped, queue_size=10)
top_ball_mask_pub = rospy.Publisher('/ball_detection/top_camera/ball_mask', Image, queue_size=10)
side_ball_mask_pub = rospy.Publisher('/ball_detection/side_camera/ball_mask', Image, queue_size=10)

#CONSTANTS
TOP_CORNERS = [[1.324277,0.630768],[1.336613,-3.374185],[-1.341470,0.640994],[-1.342677,-3.374185]] #top left, top right, bottom left, bottom right
SIDE_BOUNDS = [2.295629,-0.77859] #top and bottom poses
RESOLUTION = [720.0,480.0] #w,h

top_frame = 'top_camera_frame'
side_frame = 'side_camera_frame'

def least_squares_triangulate(x_top, x_side, top_intrinsic, side_intrinsic):
    """
    Computes the coordinates of the point represented by the corresponding pair (x1, x2).
    x1, x2 are given in unnormalized homogeneous coordinates.
    You should compute the coordinates X of the point written in the reference frame of the
    right camera.

    (R, T) is the transform g_21.

    left_intrinsic and right_intrinsic are both numpy arrays of size (3, 3) representing the 
    3x3 intrinsic matrices of the left and right cameras respectively.
    """

    top_intrinsic_inv = np.linalg.inv(top_intrinsic)
    side_intrinsic_inv = np.linalg.inv(side_intrinsic)

    # Transform from top frame to side frame
    trans = tfBuffer.lookup_transform(side_frame, top_frame, rospy.Time())
    T, quat = numpify(trans.transform.translation).reshape(-1, 1), numpify(trans.transform.rotation)
    R = np.array(transformations.quaternion_matrix(quat))[:3, :3]
    
    # Use least squares to solve for lambda1 and lambda2.

    A = np.concatenate((-R.dot(top_intrinsic_inv).dot(x_top).reshape(-1, 1), side_intrinsic_inv.dot(x_side).reshape(-1, 1)), axis=1)
    lambda_1, lambda_2 = np.linalg.lstsq(A, T.squeeze())[0]

    if lambda_1 > 0 and lambda_2 > 0:
        X2 = lambda_2 * right_intrinsic_inv.dot(x_side)
        X1 = lambda_1 * R.dot(left_intrinsic_inv).dot(x_top) + T
        X = .5 * (X1 + X2)
    else:
        return None
    
    # Transform from side frame to world
    trans = tfBuffer.lookup_transform('world', side_frame, rospy.Time())
    T, quat = ros_numpy.numpify(trans.transform.translation).reshape(-1, 1), ros_numpy.numpify(trans.transform.rotation)
    R = np.array(transformations.quaternion_matrix(quat))[:3, :3]
    X_world = R.dot(X) + T
    return X_world

count = 0

def image_callback(*msgs):
    top_rgb_image, side_rgb_image = msgs[:2]
    top_camera_info, side_camera_info = msgs[2:4]
    if publish_error:
        ground_truth_pose = msgs[4]

    global count

    estimated_pose = PointStamped()
    estimated_pose.header.frame_id = 'world'
    estimated_pose.header.stamp = top_rgb_image.header.stamp

    estimated_x = None
    estimated_y = None
    estimated_z = None

    lower_orange = np.array([0,100,100])
    upper_orange = np.array([50,255,255])

    top_frame = CvBridge().imgmsg_to_cv2(top_rgb_image, desired_encoding="bgr8")
    side_frame = CvBridge().imgmsg_to_cv2(side_rgb_image, desired_encoding="bgr8")

    top_hsv = cv2.cvtColor(top_frame, cv2.COLOR_BGR2HSV)
    side_hsv = cv2.cvtColor(side_frame, cv2.COLOR_BGR2HSV)

    top_mask = cv2.inRange(top_hsv,lower_orange, upper_orange)
    side_mask = cv2.inRange(side_hsv,lower_orange, upper_orange)

    top_masked = np.expand_dims(top_mask, -1) * top_frame
    side_masked = np.expand_dims(side_mask, -1) * side_frame

    top_interest_pixels=cv2.findNonZero(top_mask)
    side_interest_pixels=cv2.findNonZero(side_mask)

    x_top, x_side = None, None

    #ball perception top image
    if top_interest_pixels is not None and len(top_interest_pixels) > 0:
        #print(len(top_interest_pixels))
        x_max = max(top_interest_pixels,key= lambda p: p[0][0])[0][0]
        x_min = min(top_interest_pixels,key= lambda p: p[0][0])[0][0]
        y_max = max(top_interest_pixels,key= lambda p: p[0][1])[0][1]
        y_min = min(top_interest_pixels,key= lambda p: p[0][1])[0][1]

        c_x = x_min+int((x_max - x_min) / 2)
        c_y = y_min+int((y_max - y_min) / 2)
        x_top = np.array([c_x, c_y, 1])

        display_radius = int(x_max-x_min)
        top_frame = cv2.circle(top_frame, (c_x,c_y), radius=int(display_radius*1.5), color=(0, 0, 255), thickness=int(display_radius/3))

        #make the pose estimations
        avg_max_pose_x = np.mean([TOP_CORNERS[0][0],TOP_CORNERS[1][0]]) #top left and top right x values
        avg_min_pose_x = np.mean([TOP_CORNERS[2][0],TOP_CORNERS[2][0]]) #bottom left and bottom right x values
        avg_max_pose_y = np.mean([TOP_CORNERS[0][1],TOP_CORNERS[2][1]]) #top left and bottom left y values
        avg_min_pose_y = np.mean([TOP_CORNERS[1][1],TOP_CORNERS[3][1]]) #top right and bottom right y values

        estimated_x = float(avg_max_pose_x - (c_y / RESOLUTION[1]) * (avg_max_pose_x-avg_min_pose_x))
        estimated_y = float(avg_max_pose_y - (c_x / RESOLUTION[0]) * (avg_max_pose_y - avg_min_pose_y))

    #ball perception side image
    if side_interest_pixels is not None and len(side_interest_pixels) > 0:
        #print(len(side_interest_pixels))
        x_max = max(side_interest_pixels,key= lambda p: p[0][0])[0][0]
        x_min = min(side_interest_pixels,key= lambda p: p[0][0])[0][0]
        y_max = max(side_interest_pixels,key= lambda p: p[0][1])[0][1]
        y_min = min(side_interest_pixels,key= lambda p: p[0][1])[0][1]

        c_x = x_min+int((x_max - x_min) / 2)
        c_y = y_min+int((y_max - y_min) / 2)
        x_side = np.array([c_x, c_y, 1])

        display_radius = int(x_max-x_min)
        side_frame = cv2.circle(side_frame, (c_x,c_y), radius=int(display_radius*1.5), color=(0, 0, 255), thickness=int(display_radius/3))

        #make pose estimations
        upper_bound = SIDE_BOUNDS[0]
        lower_bound = SIDE_BOUNDS[1]
        estimated_z = float(upper_bound - (c_y / RESOLUTION[1]) * (upper_bound - lower_bound))
   
    top_out_image = CvBridge().cv2_to_imgmsg(top_frame, encoding="bgr8")
    side_out_image = CvBridge().cv2_to_imgmsg(side_frame, encoding="bgr8")
    # top_masked_image = CvBridge().cv2_to_imgmsg(top_masked, encoding='bgr8')
    # side_masked_image = CvBridge().cv2_to_imgmsg(side_masked, encoding='bgr8')

    top_img_pub.publish(top_out_image)
    side_img_pub.publish(side_out_image)
    # top_ball_mask_pub.publish(top_masked_image)
    # side_ball_mask_pub.publish(side_masked_image)

    #publishes only if there is new pose estimated data (image publisher not affected)
    if estimated_x != None and estimated_y != None and estimated_z != None:
        estimated_pose.point.x = estimated_x
        estimated_pose.point.y = estimated_y
        estimated_pose.point.z = estimated_z

        print("Estimated pose:", estimated_x, estimated_y, estimated_z)
        ball_pose_pub.publish(estimated_pose)

        top_intrinsic = np.array(top_camera_info.K).reshape(3, 3)
        side_intrinsic = np.array(side_camera_info.K).reshape(3, 3)
        triangulated = least_squares_triangulate(x_top, x_side, top_intrinsic, side_intrinsic)
        print("Triangulated point:", triangulated)

        if publish_error:
            pose_error = PointStamped()
            pose_error.header.frame_id = 'world'
            pose_error.header.stamp = top_rgb_image.header.stamp
            pose_error.point.x = ground_truth_pose.pos.x - estimated_x
            pose_error.point.y = ground_truth_pose.pos.y - estimated_y
            pose_error.point.z = ground_truth_pose.pos.z - estimated_z
            pose_error_pub.publish(pose_error)
            print("pose error:", np.linalg.norm(numpify(pose_error.point)))
        
        count += 1


if __name__ == '__main__':
    publish_error = len(sys.argv) > 1 and sys.argv[1] == '-e'
    rospy.init_node('ball_tracker_node', anonymous=True)
    top_camera_sub = message_filters.Subscriber('/pp/top_camera/image_raw',Image)
    side_camera_sub = message_filters.Subscriber('/pp/side_camera/image_raw',Image)
    top_camera_info_sub = message_filters.Subscriber('/pp/top_camera/camera_info', CameraInfo)
    side_camera_info_sub = message_filters.Subscriber('/pp/side_camera/camera_info', CameraInfo)
    ground_truth_sub = message_filters.Subscriber('/ball_detection/ground_truth', PosVelTimed)
    subs = [top_camera_sub, side_camera_sub, top_camera_info_sub, side_camera_info_sub]
    if publish_error:
        subs.append(ground_truth_sub)
        pose_error_pub = rospy.Publisher('/ball_detection/pose_error', PointStamped, queue_size=10)

    tfBuffer = tf2.Buffer()
    listener = tf2.TransformListener(tfBuffer)
    ts = message_filters.ApproximateTimeSynchronizer(subs, 30, .01)
    ts.registerCallback(image_callback)
    rospy.spin()
