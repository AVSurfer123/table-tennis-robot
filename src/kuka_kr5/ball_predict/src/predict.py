#! /usr/bin/python

import rospy
import numpy as np
from ball_predict.msg import PosVelTimed
import math

class EndPosVelPrediction:

	table_height = 0.76
	table_length = 2.74
	table_width = 1.525
	e = 0.83 				# coefficient of restitution
	ball_raidus = 0.02
	g = -9.81

	robot_pos_y = 0.5
	# define the end position of the ball in y-axis
	y_end = -0.15			
	ws_radius = 1.6
	# squared value for the max x position that the robot can reach at the defined end y position
	x_limit_squared = ws_radius**2 - (abs(robot_pos_y) + abs(y_end))**2	


	def __init__(self, pos_vel_sub_topic, pos_vel_pub_topic):
		# pos_vel_sub_topic: topic to get the staring time(ros simulation time), initial position 
		#	and velocity of a ball for prediciton
		# pos_vel_pub_topic: topic to publish the predicted final positon, velocity and
		#	ros simulation time
		self.sub = rospy.Subscriber(pos_vel_sub_topic, PosVelTimed, self.callback)
		self.pub = rospy.Publisher(pos_vel_pub_topic, PosVelTimed, queue_size = 10)


	# publish the state immediately if not hittable
	def pubNotHittable(self, pred_state):
		pred_state.hittable = False
		self.pub.publish(pred_state)


	# Once receive the initial state of the ball, calculate the predicted end state and publish
	def callback(self, data):
		t_initial = data.stamp.secs + data.stamp.nsecs/1e9
		x = data.pos.x
		y = data.pos.y
		z = data.pos.z
		vx = data.vel.x
		vy = data.vel.y
		vz = data.vel.z

		pred_state = PosVelTimed()
		pred_state.pos.y = self.y_end
		pred_state.vel.x = vx
		pred_state.vel.y = vy
		pred_state.hittable = True

		# check if the ball is not on the table initially
		if (z < self.table_height+self.ball_raidus
				or abs(x) > self.table_width/2
				or abs(y) > self.table_length or y > 0):
			self.pubNotHittable(pred_state)
			print('not on table\n')
			return

		#time for ball to reach the y end position
		t = abs(y - self.y_end) / vy
		pred_state.stamp.secs = int(math.modf(t_initial + t)[1])
		pred_state.stamp.nsecs = math.modf(t_initial + t)[0] * 1e9

		# check if the final x position is out of the robot workspace
		x_end = x + vx * t
		if x_end**2 > self.x_limit_squared:
			self.pubNotHittable(pred_state)
			return
		else:
			pred_state.pos.x = x_end

		# calculate the final z position
		z_temp = z + vz * t + 0.5 * self.g * t**2
		if z_temp > self.table_height+self.ball_raidus:
			# ball reaches the final y position without rebound
			#	set to hittable in case the initial state of the ball is after rebound
			pred_state.pos.z = z_temp
			pred_state.vel.z = vz + self.g*t
			self.pub.publish(pred_state)
			print("ball will not rebound but hittable\n")
			print("sent data: ")
			print(pred_state, "\n")
			return

		else:
			# ball rebounds at least onece

			# find the time for the first rebound
			t_rb1 = (-vz - math.sqrt(vz**2 - 2 * self.g * (z - self.table_height - self.ball_raidus))) / self.g
			# find the z velocity just before rebound
			vz_in = -math.sqrt(-2 * self.g * (z - self.table_height - self.ball_raidus) + vz**2)
			# find the z velocity just after rebound
			vz_out = -self.e * vz_in

			# time for second rebound
			t_rb2 = 2 * vz_out / -self.g
			# time remains from the first rebound
			t_rem = t - t_rb1
			if t_rb2 < t_rem:
				# second rebound before reaching y end position
				# NOTE: this mothed may failed if z end position is too close to the table, may need an offset (t_rb2 < t_rem - 0.2?)
				self.pubNotHittable(pred_state)
				print('rebound more than once\n')
				return
			else:
				pred_state.pos.z = z+vz_out*t_rem + 0.5*self.g*t_rem**2
				pred_state.vel.z = vz_out + self.g*t_rem
				self.pub.publish(pred_state)
				print("ball rebounds once and hittable\n")
				print("sent data: ")
				print(pred_state, "\n")
				return



def main():
	rospy.init_node('ball_predict_node', anonymous=True)
	EndPosVelPrediction("ball_detection/ball_state_filtered", "/ball_predict/ball_end_state")
	rospy.spin()


if __name__ == '__main__':
   main()