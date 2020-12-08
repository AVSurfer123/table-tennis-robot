import rospy
from moveit_msgs.msg import CollisionObject

rospy.init_node('testing')

pub = rospy.Publisher('/collision_object', CollisionObject)
rospy.spin()