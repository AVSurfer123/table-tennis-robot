#! /usr/bin/python

from __future__ import print_function

import rospy
import numpy as np
from ball_detection.msg import PosVelTimed
import math

class EndPosVelPrediction:

	table_height = 0.76
	table_length = 2.74
	table_width = 1.525
	e = 0.87 				# coefficient of restitution
	ball_radius = 0.02
	g = -9.81

	robot_pos_y = 0.5
	# define the end position of the ball in y-axis
	y_end = -0.15	
	z_end = 0.9		
	ws_radius = 1.6
	# squared value for the max x position that the robot can reach at the defined end y position
	x_limit_squared = ws_radius**2 - (abs(robot_pos_y) - abs(y_end))**2	


	def __init__(self, pos_vel_sub_topic, pos_vel_pub_topic):
		# pos_vel_sub_topic: topic to get the staring time(ros simulation time), initial position 
		#	and velocity of a ball for prediciton
		# pos_vel_pub_topic: topic to publish the predicted final positon, velocity and
		#	ros simulation time
		self.sub = rospy.Subscriber(pos_vel_sub_topic, PosVelTimed, self.callback)
		self.pub = rospy.Publisher(pos_vel_pub_topic, PosVelTimed, queue_size = 10)


	# publish the state immediately if not hittable
	def pubNotHittable(self):
		pred_state = PosVelTimed(hittable=False)
		self.pub.publish(pred_state)


	# Once receive the initial state of the ball, calculate the predicted end state and publish
	def callback(self, data):
		t_initial = data.header.stamp.secs + data.header.stamp.nsecs/1e9
		x = data.pos.x
		y = data.pos.y
		z = data.pos.z
		vx = data.vel.x
		vy = data.vel.y
		vz = data.vel.z
		y_end = self.y_end
		z_end = self.z_end

		y_net = -1.37
		y_predict = -1.6

		if vy == 0:
			self.pubNotHittable()
			print('ball not moving\n')
			return

		# check if the ball is not on the table initially
		if (z < self.table_height+self.ball_radius
				or abs(x) > self.table_width/2
				or abs(y) > self.table_length or y > 0):
			self.pubNotHittable()
			print('not on table\n')
			return

		if y < y_predict:
			self.pubNotHittable()
			print("too early to move\n")
			return

		#time for ball to reach the y end position
		t = (y_end - y) / vy
		if t < 0:
			self.pubNotHittable()
			print("ball moving in wrong direction\n")
			return

		# check if the final x position is out of the robot workspace
		x_end = x + vx * t
		if x_end**2 > self.x_limit_squared:
			self.pubNotHittable()
			print('final x-position outside of workspace:', x_end)
			return

		# calculate the final z position
		z_temp = z + vz * t + 0.5 * self.g * t**2
		if z_temp > self.table_height+self.ball_radius:
			# ball reaches the final y position without rebound
			#	set to hittable in case the initial state of the ball is after rebound
			# TODO y-position might be too much so ball bounces on very back of table
			# pred_state.pos.z = z_temp
			# pred_state.vel.z = vz + self.g*t
			pred_state = PosVelTimed()
			pred_state.pos.x = x_end
			pred_state.pos.y = y_end
			pred_state.pos.z = z_temp
			pred_state.vel.x = vx
			pred_state.vel.y = vy
			pred_state.vel.z = vz + t * self.g
			pred_state.hittable = True
			pred_state.header.frame_id = 'world'
			fractional, integer = math.modf(t_initial + t)
			pred_state.header.stamp.secs = int(integer)
			pred_state.header.stamp.nsecs = fractional * 1e9
			self.pub.publish(pred_state)
			print("ball did not hit the table or used estimated state after rebound")
			print('goal position:', x_end, y_end, z_end, '\n')
			return

		# Otherwise, ball rebounds at least once

		# find the time for the first rebound
		t_rb1 = (-vz - math.sqrt(vz**2 - 2 * self.g * (z - self.table_height - self.ball_radius))) / self.g
		# find the z velocity just before rebound
		vz_in = -math.sqrt(-2 * self.g * (z - self.table_height - self.ball_radius) + vz**2)
		# find the z velocity just after rebound
		vz_out = -self.e * vz_in

		# time for second rebound
		t_rb2 = 2 * vz_out / -self.g
		# time remains from the first rebound
		t_rem = t - t_rb1
		if t_rb2 < t_rem:
			# second rebound before reaching y end position
			# NOTE: this method may fail if z end position is too close to the table, may need an offset (t_rb2 < t_rem - 0.2?)
			
			# Quadratic equation z_end = v_z t + .5 a * t^2
			try:
				discrim = math.sqrt(vz_out ** 2 + 2*(self.g)* (z_end - self.table_height - self.ball_radius))
				print(vz_out, z_end)
			except:
				print("cannot find solution")
				return 
			time1, time2 = (-vz_out + discrim)/(-self.g), (-vz_out - discrim)/(-self.g)
			t_hit = None
			choose1, choose2 = True, True
			# time of hit must occur after bounce
			# time of hit must occur before ball goes off table
			print("rebound more than once. quadratic solutions for hitting time:", time1, time2)
			if time1 < 0 or time1 > t_rem or abs(y + (t_rb1 + time1) * vy) > self.ws_radius:
				choose1 = False
			if time2 < 0 or time2 > t_rem or abs(y + (t_rb1 + time2) * vy) > self.ws_radius:
				choose2 = False
			if choose1 == False and choose2 == False:
				print("quadratic didn't find any valid solutions")
				self.pubNotHittable()
				return
			elif choose1 == True and choose2 == True:
				print("quadratic has 2 valid solutions. chose the larger one")
				t_hit = max(time1, time2)
			elif choose1 == True:
				t_hit = time1
			elif choose2 == True:
				t_hit = time2
			
			t_end = t_rb1 + t_hit
			vz_end = vz_out + self.g * t_hit
			x_end = x + vx*t_end
			y_end = y + vy*t_end		
		else:
			z_end = self.table_height + self.ball_radius +vz_out*t_rem + 0.5*self.g*t_rem**2
			vz_end = vz_out + self.g*t_rem
			print("ball rebounds once and hittable")
			print('goal position:', x_end, y_end, z_end, '\n')
		
		pred_state = PosVelTimed()
		pred_state.pos.x = x_end
		pred_state.pos.y = y_end
		pred_state.pos.z = z_end
		pred_state.vel.x = vx
		pred_state.vel.y = vy
		pred_state.vel.z = vz_end
		pred_state.hittable = True
		pred_state.header.frame_id = 'world'
		fractional, integer = math.modf(t_initial + t)
		pred_state.header.stamp.secs = int(integer)
		pred_state.header.stamp.nsecs = fractional * 1e9
		
		self.pub.publish(pred_state)


if __name__ == '__main__':
	rospy.init_node('ball_predict_node', anonymous=True)
	predictor = EndPosVelPrediction("/ball_detection/ball_state_filtered", "/ball_detection/predicted_ball_state")
	rospy.spin()
