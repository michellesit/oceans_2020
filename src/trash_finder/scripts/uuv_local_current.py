#!/usr/bin/env python

##get the robot's pose and depth
##subscribe to the robot's ground_truth_to_tf_[name of robot]/pose
import rospy
from geometry_msgs.msg import PoseStamped

from scipy.io import netcdf
import numpy as np
from haversine_dist import calc_dist_lat_lon
from scipy.interpolate import interpn

from uuv_world_ros_plugins_msgs.srv import SetCurrentModel, SetCurrentVelocity
import math

import pdb

class UUV_local_current():

    def __init__(self, current_file, bbox1, bbox2, bbox3, bbox4):
        self.current_file = current_file

        self.lon = current_file.variables['lon']
        self.lon = self.lon[:] - 360
        self.lat = current_file.variables['lat']

        self.d = current_file.variables['depth']
        self.u = current_file.variables['u']
        self.v = current_file.variables['v']

        self.width, self.height, self.min_lat, self.max_lat, self.min_lon, self.max_lon = \
                self.calc_param(bbox1, bbox2, bbox3, bbox4)

        ##get the idx values from lat, lon arrays
        self.lon_idx = np.where((self.lon[:]>=self.min_lon) & (self.lon[:]<=self.max_lon))[0]
        self.lat_idx = np.where((self.lat[:]>=self.min_lat) & (self.lat[:]<=self.max_lat))[0]

        lon_area = self.lon[self.lon_idx]
        lon_zeroed = lon_area - min(self.lon[self.lon_idx])
        self.lon_xcoords = lon_zeroed*(self.width/max(lon_zeroed))*1000

        lat_area = self.lat[self.lat_idx]
        lat_zeroed = lat_area - min(self.lat[self.lat_idx])
        self.lat_ycoords = lat_zeroed*(self.height/max(lat_zeroed))*1000


    def callback(self, msg):
    	position = msg.pose.position
    	uuv_pos = [position.x, position.y, abs(position.z)]
    	print ("UUV_POS: ", uuv_pos)
    	# rospy.loginfo("Point position: [%f, %f, %f]" %(position.x, position.y, position.z))

    	uuv_u, uuv_v = self.convert_to_xy(self.current_file, uuv_pos)

    	print ("UUV_U: ", uuv_u)
    	print ("UUV_V: ", uuv_v)

    	##send data via ROS
    	self.update_hydrodynamic(uuv_u, uuv_v, "something")


    def listener(self):
    	rospy.init_node('get_robot_pose', anonymous=True)
    	rospy.Subscriber("rexrov/ground_truth_to_tf_rexrov/pose", PoseStamped, self.callback)

    	rospy.spin()


    ##TODO: Unit test to make sure this makes sense for all data points (all edge cases)
    ##CASE: Data point has no info
    ##CASE: None of the data points have info
    ##CASE: At the edge of the map
    ##CASE: At the very bottom of the ground

    def convert_to_xy(self, current_file, uuv_pos):
        [uuv_x, uuv_y, uuv_z] = uuv_pos
        uuv_x += self.width/2
        uuv_y += self.height/2

        ##figure out between which idx values the coordinates are in
        depth_idx_pt = self.find_idx_boundary(uuv_z, self.d)
        lon_idx_pt = self.find_idx_boundary(uuv_x, self.lon_xcoords)
        lat_idx_pt = self.find_idx_boundary(uuv_y, self.lat_ycoords)

        print( "depth shape: ", self.d.shape)
        print ("lat_idx shape: ", self.lat_idx.shape)
        print ("lon_idx shape: ", self.lon_idx.shape)

        print ("depth_idx_pt: ", depth_idx_pt)
        print ("lat_idx_pt: ", lat_idx_pt)
        print ("lon_idx_pt: ", lon_idx_pt)

        ##Use those idx values to get depth values from u and v
        ##U and V are read-only(?). Hence, copy the values

        # print (lat_idx[lat_idx_pt[0]], lat_idx[lat_idx_pt[-1]])
        # print (lon_idx[lon_idx_pt[0]], lon_idx[lon_idx_pt[-1]])
        u_above = self.u[:, depth_idx_pt[1]+1, self.lat_idx[lat_idx_pt[0]]:self.lat_idx[lat_idx_pt[1]+1], self.lon_idx[lon_idx_pt[0]]:self.lon_idx[lon_idx_pt[1]+1]].copy()
        v_above = self.v[:, depth_idx_pt[1]+1, self.lat_idx[lat_idx_pt[0]]:self.lat_idx[lat_idx_pt[1]+1], self.lon_idx[lon_idx_pt[0]]:self.lon_idx[lon_idx_pt[1]+1]].copy()

        u_below = self.u[:, depth_idx_pt[0], self.lat_idx[lat_idx_pt[0]]:self.lat_idx[lat_idx_pt[1]+1], self.lon_idx[lon_idx_pt[0]]:self.lon_idx[lon_idx_pt[1]+1]].copy()
        v_below = self.v[:, depth_idx_pt[0], self.lat_idx[lat_idx_pt[0]]:self.lat_idx[lat_idx_pt[1]+1], self.lon_idx[lon_idx_pt[0]]:self.lon_idx[lon_idx_pt[1]+1]].copy()

        print ("u")
        print (u_above)
        print (u_below)

        print ("v")
        print (v_above)
        print (v_below)

        u_box = np.vstack((u_below, u_above))
        v_box = np.vstack((v_below, v_above))

        ##3D interpolate the values from the coordinates here
        ##get the individual u,v coordiantes based off of these things
        ##Also need a horizontal component (z direction)
        lat_box = np.array([self.lat_ycoords[lat_idx_pt[0]], self.lat_ycoords[lat_idx_pt[1]]])
        lon_box = np.array([self.lon_xcoords[lon_idx_pt[0]], self.lon_xcoords[lon_idx_pt[1]]])
        depth_box = np.array([self.d[depth_idx_pt[0]], self.d[depth_idx_pt[1]+1]])

        print ("lat_box: ", lat_box)
        print ("lon_box: ", lon_box)
        print ("depth_box: ", depth_box)

        ##Coordinate space, the actual values, the pts you're trying to interpolate
        uuv_u = interpn((lon_box, lat_box, depth_box), u_box, np.array([uuv_x, uuv_y, uuv_z]).T)
        uuv_v = interpn((lon_box, lat_box, depth_box), v_box, np.array([uuv_x, uuv_y, uuv_z]).T)
        uuv_z = interpn((lon_box, lat_box, depth_box), v_box, np.array([uuv_x, uuv_y, uuv_z]).T)

        print ("uuv_u: ", uuv_u)
        print ("uuv_v: ", uuv_v)

        return uuv_u[0], uuv_v[0]
        

    ##What do we want to send this topic?
    def update_hydrodynamic(self, input_u, input_v, topic_name):
    	rospy.wait_for_service('/rexrov/set_current_velocity/')

    	##Calculate the magnitude and use for uv value
    	uv = np.hypot(input_u, input_v)
    	print ("uv: ", uv)

    	##Calculate the horizontal angle of this vector:
    	uv_angle = math.atan2(input_u, input_v)
    	print ("uv_angle: ", uv_angle)

    	# send_uv = rospy.ServiceProxy('/rexrov/set_current_velocity_model', SetCurrentModel)
    	send_uv = rospy.ServiceProxy('/rexrov/set_current_velocity', SetCurrentVelocity)
    	response = send_uv(uv, uv_angle, 0)
    	print ("response: ", response)


    ##Helper method
    def find_idx_boundary(self, find_value, data_arr):
    	max_idx = np.where(find_value <= data_arr[:])[0]
    	min_idx = np.where(find_value >= data_arr[:])[0]

    	return max(min_idx), min(max_idx)


    def calc_param(self, loc1, loc2, loc3, loc4):

        ##loc1 to loc2 (width)
        width = calc_dist_lat_lon(loc1[0], loc1[1], loc2[0], loc2[1])
        ##loc2 to loc 3 (height)
        height = calc_dist_lat_lon(loc2[0], loc2[1], loc3[0], loc3[1])

        min_lat = min(loc1[0], loc2[0], loc3[0], loc4[0])
        max_lat = max(loc1[0], loc2[0], loc3[0], loc4[0])
        min_lon = min(loc1[1], loc2[1], loc3[1], loc4[1])
        max_lon = max(loc1[1], loc2[1], loc3[1], loc4[1])

        return width, height, min_lat, max_lat, min_lon, max_lon


if __name__ == '__main__':
	##Calculate the current at that position
	current_db = netcdf.NetCDFFile('/home/msit/Desktop/oceans_2020/src/trash_finder/scripts/ca_subCA_das_2020010615.nc')

	##we'll pretend that this info is being fed in:
	lajolla = [32.838, -117.343]
	lajolla_20 = [32.838, -117.543]
	coronado = [32.66, -117.343]
	coronado_20 = [32.66, -117.543]

	##Get robot position in Gazebo
	C = UUV_local_current(current_db, lajolla, lajolla_20, coronado, coronado_20)
	C.listener()


	# print ("UUV_POS: ", uuv_pos)

	# # uuv_pos = [5000, 6000, 150]
	# uuv_u, uuv_v = convert_to_xy(lajolla, lajolla_20, coronado, coronado_20, current_file, uuv_pos)

	# print ("UUV_U: ", uuv_u)
	# print ("UUV_V: ", uuv_v)

	# ##send data via ROS
	# update_hydrodynamic(uuv_u, uuv_v, "something")

