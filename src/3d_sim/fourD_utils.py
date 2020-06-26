import pickle
from math import atan2, sin, cos, acos

import rospkg
from scipy.io import netcdf
import numpy as np
import shapely.geometry as sg
import shapely.affinity as sa
from shapely.ops import nearest_points
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy.linalg import norm

from trash_utils.haversine_dist import haversine
from trash_utils.finder_utils import (grid_data,
									  wpts_to_yaml,
									  load_currents)

from trash_utils.trash_lib import trash_2d_gauss, init_hotspots, visualize_trash_flow, update_trash_pos,visualize_trash_step
from trash_utils.cc_utils import calc_mowing_lawn, search_for_trash
from trash_utils.Env import Env

import pdb


'''
Hotspot path planning algorithms

'''

def follow_path_waypoints(all_waypoints, uuv):
	for pt in range(all_waypoints.shape[0]):
		##Calculate heading
		goal_pt = all_waypoints[pt]
		
		


		cost_to_waypoint_v1()




def cost_to_waypoint_v1(input_start_pos, goal_pos, goal_heading, time_now, vis_ctw=False):
	'''
	Given headings and positions, calculate the cost of getting to that position
	Returns cost as timesteps and uuv_controls

	Input:
		start_pos (np.ndarray): (x,y,z)
		goal_pos (np.ndarray): (x,y,z)
		goal_heading (int, np.ndarray): radians [phi, theta]
		time_now (int): time in seconds

	Returns:
		cost (float) : formula calculated from paper
		timesteps (int) : time it took to travel to the goal node in seconds
		controls (np.ndarray) : inputs to the system

	'''

	threshold2 = 2	##meters
	timesteps = 0

	uuv_controls = []
	pos = np.copy(input_start_pos)

	goal_phi = goal_heading[0]
	goal_theta = goal_heading[1]

	if vis_ctw == True:
		fig = plt.figure()
		ax1 = fig.gca(projection='3d')
		ax1.plot([input_start_pos[0]], [input_start_pos[1]], [input_start_pos[2]], 'bo')
		ax1.plot([goal_pos[0]], [goal_pos[1]], [goal_pos[2]], 'bo')
		ax1.plot([input_start_pos[0], goal_pos[0]], [input_start_pos[1], goal_pos[1]], [input_start_pos[2], goal_pos[2]], 'b--')

	epoch2 = 0
	max_num_epoch2 = 150
	while abs(np.linalg.norm([pos - goal_pos])) > threshold2 and epoch2 < max_num_epoch2:
		time_now_hrs = (time_now + timesteps)/3600.0
		##Calculate ocean u,v currents
		current_u = self.ufunc([time_now_hrs, abs(pos[2]), pos[0], pos[1]])[0] * self.u_boost * np.random.normal(1, 0.5)
		current_v = self.vfunc([time_now_hrs, abs(pos[2]), pos[0], pos[1]])[0] * self.v_boost * np.random.normal(1, 0.5)

		##Calculate the desired heading and position of the uuv
		mid_goal_x = self.desired_speed * sin(goal_phi) * cos(goal_theta)
		mid_goal_y = self.desired_speed * sin(goal_phi) * sin(goal_theta)
		mid_goal_z = self.desired_speed * cos(goal_phi)

		##Adapt the mid_goal in respose to distance to actual point
		if mid_goal_x > abs(np.linalg.norm([pos[0] - goal_pos[0]])):
			mid_goal_x = abs(np.linalg.norm([pos[0] - goal_pos[0]]))
		if mid_goal_y > abs(np.linalg.norm([pos[1] - goal_pos[1]])):
			mid_goal_y = abs(np.linalg.norm([pos[1] - goal_pos[1]]))
		if mid_goal_z > abs(np.linalg.norm([pos[2] - goal_pos[2]])):
			mid_goal_z = abs(np.linalg.norm([pos[2] - goal_pos[2]]))

		##If mid_goal is outside the bounds of the map, then set to the edge
		if abs(pos[0]+mid_goal_x) > self.width/2:
			mid_goal_x = 0.0
		if abs(pos[1]+mid_goal_y) > self.height/2:
			mid_goal_y = 0.0
		if abs(pos[2]+mid_goal_z) > self.max_depth:
			mid_goal_z = 0.0

		##Calculate the needed UUV offset and at what angle
		desired_x = mid_goal_x - current_u
		desired_y = mid_goal_y - current_v
		desired_z = mid_goal_z
		uuv_vector = np.linalg.norm([desired_x, desired_y, desired_z])

		# ##TODO: throw in check. If uuv_vector > possible result, return 9999999999 cost
		if uuv_vector > self.max_uuv_vector:
			print ("uuv_vector: ", uuv_vector)
			return np.inf, np.inf, np.empty((0,6))

		uuv_phi = acos((desired_z / self.desired_speed))
		uuv_theta = atan2(desired_y, desired_x)

		uuv_controls.append([uuv_vector, desired_x, desired_y, desired_z, uuv_phi, uuv_theta])
		timesteps += 1

		##update pos with resulting location
		pos += [mid_goal_x, mid_goal_y, mid_goal_z]
		epoch2 += 1
		# print ("epoch2: ", epoch2)

		#Visualize this makes sense
		if vis_ctw:
			ax1.plot([pos[0]], [pos[1]], [pos[2]], 'ro')

	if vis_ctw:
		ax1.set_xlabel("x-axis")
		ax1.set_ylabel("y-axis")
		ax1.set_zlabel("depth")
		plt.show()

	if not uuv_controls:
		time_now_hrs = (time_now + timesteps)/3600.0
		##Calculate ocean u,v currents
		current_u = self.ufunc([time_now_hrs, abs(pos[2]), pos[0], pos[1]])[0] * self.u_boost * np.random.normal(1, 0.5)
		current_v = self.vfunc([time_now_hrs, abs(pos[2]), pos[0], pos[1]])[0] * self.v_boost * np.random.normal(1, 0.5)

		##Calculate the desired heading and position of the uuv
		mid_goal_x = self.desired_speed * sin(goal_phi) * cos(goal_theta)
		mid_goal_y = self.desired_speed * sin(goal_phi) * sin(goal_theta)
		mid_goal_z = self.desired_speed * cos(goal_phi)

		##Adapt the mid_goal in respose to distance to actual point
		if mid_goal_x > abs(np.linalg.norm([pos[0] - goal_pos[0]])):
			mid_goal_x = abs(np.linalg.norm([pos[0] - goal_pos[0]]))
		if mid_goal_y > abs(np.linalg.norm([pos[1] - goal_pos[1]])):
			mid_goal_y = abs(np.linalg.norm([pos[1] - goal_pos[1]]))
		if mid_goal_z > abs(np.linalg.norm([pos[2] - goal_pos[2]])):
			mid_goal_z = abs(np.linalg.norm([pos[2] - goal_pos[2]]))

		##If mid_goal is outside the bounds of the map, then set to the edge
		if abs(pos[0] + mid_goal_x) > self.width/2:
			mid_goal_x = 0.0
		if abs(pos[1] + mid_goal_y) > self.height/2:
			mid_goal_y = 0.0
		if abs(pos[2]+mid_goal_z) > self.max_depth:
			mid_goal_z = 0.0

		##Calculate the needed UUV offset and at what angle
		desired_x = mid_goal_x - current_u
		desired_y = mid_goal_y - current_v
		desired_z = mid_goal_z
		uuv_vector = np.linalg.norm([desired_x, desired_y, desired_z])
		# print ("uuv_vector: ", uuv_vector)

		# ##TODO: throw in check. If uuv_vector > possible result, return 9999999999 cost
		# if uuv_vector > self.max_uuv_vector:
		# 	return 99999999999999999, 99999999999999999999, np.empty((0,6))

		uuv_phi = acos((desired_z / self.desired_speed))
		uuv_theta = atan2(desired_y, desired_x)

		uuv_controls = [uuv_vector, desired_x, desired_y, desired_z, uuv_phi, uuv_theta]
		cost = (timesteps*(12.0/3600.0)) + (timesteps * ((uuv_controls[0]**3)**(0.333333))) * 0.30
	else:
		uuv_controls = np.array(uuv_controls)
		cost = (timesteps*(12.0/3600.0)) + (timesteps * (np.sum((uuv_controls[:, 0]**3))**(0.333333))) * 0.30


	if epoch2 >= max_num_epoch2:
		return np.inf, np.inf, np.array(uuv_controls).reshape((-1, 6))
	else:
		return cost, timesteps, np.array(uuv_controls).reshape((-1, 6))