#! /usr/bin/python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

pub = rospy.Publisher('/pp/camera1/image_filtered', Image, queue_size=1)
out_image = Image()
new_data = False

def image_callback(rgb_msg):
   global out_image, new_data

   lower_orange = np.array([0,100,100])
   upper_orange = np.array([255,255,255])

   frame = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")

   hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   mask = cv2.inRange(hsv,lower_orange,upper_orange)
   res = cv2.bitwise_and(frame,frame,mask=mask)

   interest_pixels=cv2.findNonZero(mask)

   x_max = max(interest_pixels,key= lambda p: p[0][0])[0][0]
   x_min = min(interest_pixels,key= lambda p: p[0][0])[0][0]
   y_max = max(interest_pixels,key= lambda p: p[0][1])[0][1]
   y_min = min(interest_pixels,key= lambda p: p[0][1])[0][1]
   #print(x_min,x_max,y_min,y_max)
   c_x = x_min+int((x_max - x_min) / 2)
   c_y = y_min+int((y_max - y_min) / 2)

   display_radius = int(x_max-x_min)
   res = cv2.circle(frame, (c_x,c_y), radius=display_radius*1.5, color=(0, 0, 255), thickness=int(display_radius/3))

   out_image = CvBridge().cv2_to_imgmsg(res,encoding="bgr8")

   new_data = True
   #camera_info_K = np.array(camera_info.K).reshape([3, 3])
   #camera_info_D = np.array(camera_info.D)
   #rgb_undist = cv2.undistort(rgb_image, camera_info_K, camera_info_D)
#length of table in meters 9ft (2.7432), 5ft (1.524). 
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