#! /usr/bin/python

import rospy
from ball_predict.msg import PosVelTimed

def talker():
	pub = rospy.Publisher("/ball_detection/ball_state_filtered", PosVelTimed, queue_size = 10)
	r = rospy.Rate(1)

	while not rospy.is_shutdown():
		pub_data = PosVelTimed()
		pub_data.stamp = rospy.Time.now()
		pub_data.pos.x = 0.0
		pub_data.pos.y = -2.70
		pub_data.pos.z = 1.5
		pub_data.vel.x = 0.0
		pub_data.vel.y = 5.0
		pub_data.vel.z = 1.5
		pub.publish(pub_data)
		print("data sent: ")
		print(pub_data)
		print(" ")
		r.sleep()

if __name__ == '__main__':

  rospy.init_node('ball_pub_test_node', anonymous=True)
  
  try:
    talker()
  except rospy.ROSInterruptException: pass