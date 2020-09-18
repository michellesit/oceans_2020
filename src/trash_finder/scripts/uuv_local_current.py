#!/usr/bin/env python

import sys
import pickle
from math import atan2

import numpy as np
from scipy.io import netcdf

import rospkg
import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from uuv_world_ros_plugins_msgs.srv import SetCurrentModel, SetCurrentVelocity, SetCurrentDirection

from trash_utils.haversine_dist import haversine
from trash_utils.finder_utils import get_current_block, get_depth_block

import pdb

'''
Get the robot's pose and depth (X,Y,Z)
Subscribe to the robot's ground_truth_to_tf_[name of robot]/pose


'''

class UUV_local_current():

	def __init__(self, current_file, bbox, name):
		self.bbox = bbox
		self.current_file = current_file
		self.uuv_name = name

		self.u, self.v, self.uv, self.u_func, \
		self.v_func, self.uv_func = get_current_block(
										self.bbox, self.current_file)


	def listener(self):
		'''
		Retrieves the uuv's position
		'''
		rospy.init_node('get_robot_pose', anonymous=True)
		rospy.Subscriber('{0}/ground_truth_to_tf_{0}/pose'.format(self.uuv_name), PoseStamped, self.callback)

		rospy.spin()


	def callback(self, msg):
		'''
		Retrieves the uuv's position, converts the coordinates into xy,
			and [WHAT?]

		Input:
			msg () : TODO
		'''
		position = msg.pose.position
		uuv_pos = [position.x, position.y, abs(position.z)]
		# print ("UUV_POS: ", uuv_pos)
		# rospy.loginfo("Point position: [%f, %f, %f]" %(position.x, position.y, position.z))

		uuv_u, uuv_v = self.get_uv_from_pos(uuv_pos)

		##send data via ROS
		self.update_hydrodynamic(uuv_u, uuv_v)


	##TODO: Unit test to make sure this makes sense for all data points (all edge cases)
	##CASE: Data point has no info
	##CASE: None of the data points have info
	##CASE: At the edge of the map
	##CASE: At the very bottom of the ground

	def get_uv_from_pos(self, uuv_pos):
		'''
		Takes the uuv's x,y,z position and finds the u,v value

		Input:
			uuv_pos () : uuv's x,y,z position

		Returns:
			uuv_u (float) :
			uuv_v (float) :
		'''
		[uuv_x, uuv_y, uuv_z] = uuv_pos
		uuv_u = self.u_func([uuv_z, uuv_x, uuv_y])
		uuv_v = self.v_func([uuv_z, uuv_x, uuv_y])

		# print ("uuv_u: ", uuv_u)
		# print ("uuv_v: ", uuv_v)

		return uuv_u, uuv_v
		

	##What do we want to send this topic?
	def update_hydrodynamic(self, input_u, input_v):
		rospy.wait_for_service('/{0}/set_current_velocity/'.format(self.uuv_name))

		##Calculate the magnitude and use for uv value
		uv = np.hypot(input_u, input_v)

		##Calculate the horizontal angle of this vector:
		uv_angle = atan2(input_u, input_v)
		

		##TO FIX: 
		##md5sum mismatch making local subscription to topic /rexrov/current_velocity
		##Expects this data to be Subscriber expects type geometry_msgs/Vector3
		## Publisher provides type geometry_msgs/TwistStamped
		
		# send_uv = rospy.ServiceProxy('/rexrov/set_current_velocity_model', SetCurrentModel)
		send_uv = rospy.ServiceProxy('/{0}/set_current_velocity'.format(self.uuv_name), SetCurrentVelocity)
		
		# ##publish updates to monitor the system:
		# current_pub = rospy.Publisher('/{0}/pub_current_velocity'.format(self.uuv_name), String)
		# rospy.init_node('/{0}pub_current_velocity'.format(self.uuv_name))
		# r = rospy.Rate(50)
		# pub.publish("{0} uv value: {1}".format(self.uuv_name, uv))

		response = send_uv(3, np.deg2rad(45), 0.0)
		# response = send_uv(uv, uv_angle, 0.0)
		print ("response: ", response)


if __name__ == '__main__':
	print ("sys argv: ", sys.argv)

	rospack = rospkg.RosPack()
	data_path = rospack.get_path('data_files')
	trash_finder_path = rospack.get_path('trash_finder')
	config = pickle.load(open(trash_finder_path + "/config/demo_configs.p", "rb"))

	current_filepath = data_path + '/' + config['current_file']

	place_bbox = np.array(config['coords_bbox'])
	C = UUV_local_current(current_filepath, place_bbox, sys.argv[1])


	rospy.wait_for_service('/{0}/set_current_velocity/'.format(C.uuv_name))

	set_init_cv = rospy.ServiceProxy('/{0}/set_current_velocity_model'.format(C.uuv_name), SetCurrentModel)
	set_cv_response = set_init_cv(0.0, 0, 50, 0.0, 0.0)
	# print ("SENT_CV_RESPONSE: ", set_cv_response)

	set_hangle = rospy.ServiceProxy('/{0}/set_current_horz_angle_model'.format(C.uuv_name), SetCurrentModel)
	set_hangle_response = set_hangle(0.0, -3.141592653589793238, 3.141592653589793238, 0.10, 0.10)
	# print ("SENT_HANGLE_RESPONSE: ", set_hangle_response)

	set_vangle = rospy.ServiceProxy('/{0}/set_current_vert_angle_model'.format(C.uuv_name), SetCurrentModel)
	set_vangle_response = set_hangle(1.0, -3.141592653589793238, 3.141592653589793238, 0.10, 0.10)
	# print ("SENT_VANGLE_RESPONSE: ", set_vangle_response)



	##Get robot position in Gazebo
	C.listener()
