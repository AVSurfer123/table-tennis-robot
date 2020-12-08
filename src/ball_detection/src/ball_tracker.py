#! /usr/bin/python
import cv2
import message_filters
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import numpy as np

img_pub = rospy.Publisher('/ball_detection/camera/image_filtered', Image, queue_size=1)
ball_pose_pub = rospy.Publisher('/ball_detection/ball_pose', Pose, queue_size=1)

top_out_image = Image()
side_out_image = Image()
estimated_pose = Pose()
new_data = False
count = 0

#CONSTANTS
TOP_CORNERS = [[1.325,0.554],[1.297,-3.261],[-1.245,0.562],[-1.214,-3.269]] #top left, top right, bottom left, bottom right
SIDE_BOUNDS = [2.150,-0.149] #top and bottom poses
RESOLUTION = [720.0,480.0] #w,h

def image_callback(top_rgb_image,side_rgb_image):
   global top_out_image, side_out_image, estimated_pose, new_data
   estimated_x = None
   estimated_y = None
   estimated_z = None

   lower_orange = np.array([0,100,100])
   upper_orange = np.array([50,255,255])

   top_frame = CvBridge().imgmsg_to_cv2(top_rgb_image, desired_encoding="bgr8")
   side_frame = CvBridge().imgmsg_to_cv2(side_rgb_image, desired_encoding="bgr8")

   top_hsv = cv2.cvtColor(top_frame, cv2.COLOR_BGR2HSV)
   side_hsv = cv2.cvtColor(side_frame, cv2.COLOR_BGR2HSV)

   top_mask = cv2.inRange(top_hsv,lower_orange,upper_orange)
   side_mask = cv2.inRange(side_hsv,lower_orange,upper_orange)

   #frame = cv2.bitwise_and(frame,frame,mask=mask)
   
   top_interest_pixels=cv2.findNonZero(top_mask)
   side_interest_pixels=cv2.findNonZero(side_mask)

   #interest_pixels_filtered = list(filter(lambda p: (p[0][0] > 0), interest_pixels))

   #ball perception top image
   if top_interest_pixels != None and len(top_interest_pixels) > 0:
      #print(len(top_interest_pixels))
      x_max = max(top_interest_pixels,key= lambda p: p[0][0])[0][0]
      x_min = min(top_interest_pixels,key= lambda p: p[0][0])[0][0]
      y_max = max(top_interest_pixels,key= lambda p: p[0][1])[0][1]
      y_min = min(top_interest_pixels,key= lambda p: p[0][1])[0][1]

      c_x = x_min+int((x_max - x_min) / 2)
      c_y = y_min+int((y_max - y_min) / 2)

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
   if side_interest_pixels != None and len(side_interest_pixels) > 0:
      #print(len(side_interest_pixels))
      x_max = max(side_interest_pixels,key= lambda p: p[0][0])[0][0]
      x_min = min(side_interest_pixels,key= lambda p: p[0][0])[0][0]
      y_max = max(side_interest_pixels,key= lambda p: p[0][1])[0][1]
      y_min = min(side_interest_pixels,key= lambda p: p[0][1])[0][1]

      c_x = x_min+int((x_max - x_min) / 2)
      c_y = y_min+int((y_max - y_min) / 2)

      display_radius = int(x_max-x_min)
      side_frame = cv2.circle(side_frame, (c_x,c_y), radius=int(display_radius*1.5), color=(0, 0, 255), thickness=int(display_radius/3))

      #make pose estimations
      upper_bound = SIDE_BOUNDS[0]
      lower_bound = SIDE_BOUNDS[1]
      estimated_z = float(upper_bound - (c_y / RESOLUTION[1]) * (upper_bound - lower_bound))
   print(estimated_x,estimated_y,estimated_z)
   #publishes only if there is new pose estimated data (image publisher not affected)
   if (estimated_x != None and estimated_y != None) or estimated_z != None:
         estimated_pose.position.x = estimated_x
         estimated_pose.position.y = estimated_y
         estimated_pose.position.z = estimated_z
         new_data = True

   top_out_image = CvBridge().cv2_to_imgmsg(top_frame,encoding="bgr8")
   side_out_image = CvBridge().cv2_to_imgmsg(side_frame,encoding="bgr8")

def publisher_callback(event):
   global out_image, new_data, count
   img_pub.publish(top_out_image)
   if new_data:
      print("publish frame {}".format(count))
      ball_pose_pub.publish(estimated_pose)
      new_data = False
      count+=1

if __name__ == '__main__':
   rospy.init_node('ball_tracker_node', anonymous=True)
   top_camera_sub = message_filters.Subscriber('/pp/camera1/image_raw',Image)
   side_camera_sub = message_filters.Subscriber('/pp/side_camera/image_raw',Image)
   ts = message_filters.TimeSynchronizer([top_camera_sub,side_camera_sub],10)
   ts.registerCallback(image_callback)
   timer = rospy.Timer(rospy.Duration(0.001), publisher_callback) #publishes at ~100fps, but input is 30fps
   rospy.spin()
   timer.shutdown