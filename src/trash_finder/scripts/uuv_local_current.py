#!/usr/bin/env python

##get the robot's pose and depth
##subscribe to the robot's ground_truth_to_tf_[name of robot]/pose
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

def callback(msg):
	position = msg.pose.position
	rospy.loginfo("Point position: [%f, %f, %f]" %(position.x, position.y, position.z))
	# rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
	rospy.init_node('get_robot_pose', anonymous=True)
	rospy.Subscriber("rexrov/ground_truth_to_tf_rexrov/pose", PoseStamped, callback)

	rospy.spin()

if __name__ == '__main__':
	listener()




##Get the current velocity and direction at that depth