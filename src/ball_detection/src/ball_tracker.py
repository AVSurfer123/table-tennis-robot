#! /usr/bin/python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

pub = rospy.Publisher('/pp/camera1/image_filtered', Image, queue_size=1)
out_image = Image()
new_data = False

def image_callback(rgb_msg):
   global out_image, new_data
   rgb_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")

   #do shit with rgb image

   out_image = CvBridge().cv2_to_imgmsg(rgb_image,encoding="rgb8")

   new_data = True
   #camera_info_K = np.array(camera_info.K).reshape([3, 3])
   #camera_info_D = np.array(camera_info.D)
   #rgb_undist = cv2.undistort(rgb_image, camera_info_K, camera_info_D)

def publisher_callback(event):
   global out_image, new_data
   if new_data:
      print("publish")
      pub.publish(out_image)
      new_data = False

if __name__ == '__main__':
   rospy.init_node('ball_tracker_node', anonymous=True)
   rospy.Subscriber('/pp/camera1/image_raw', Image, image_callback)
   timer = rospy.Timer(rospy.Duration(0.001), publisher_callback) #publishes at ~100fps, but input is 30fps
   rospy.spin()
   timer.shutdown