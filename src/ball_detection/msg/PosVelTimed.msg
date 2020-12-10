#Time stamp of the position and velocity of a ball
# int32 secs
# int32 nsecs
std_msgs/Header header

#Position of the ball
geometry_msgs/Point pos

#Velocity of the ball
geometry_msgs/Vector3 vel

#Determine if the ball can be reached
bool hittable
