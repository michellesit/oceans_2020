from math import atan2, sin, cos, acos

import numpy as np
import shapely.geometry as sg
import shapely.affinity as sa
from shapely.ops import nearest_points
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from trash_utils.Env import Env
from trash_utils.UUV import UUV
from trash_utils.trash_lib import (	init_hotspots, 
									visualize_trash_flow,
									update_trash_pos,
									visualize_trash_step)
from trash_utils.cc_utils import calc_mowing_lawn, search_for_trash

from fourD_utils import cost_to_waypoint_v1, follow_path_waypoints

import pdb

'''
Deploy multiple trash hotspots in an environment

Calculate a path in 3D that would go from spot to spot

Do complete coverage in those spots
'''

class Nom_Simulation():

	def __init__(self):
		self.Env = Env() ##This now contains all the ufunc, vfunc, width, height, u_boost, v_boost
		self.num_hotspots = 6
		self.max_depth = 50

		self.uuv = UUV()
		
		#For 4D path planning
		self.desired_speed = 2.5722 	##meters/second (5 knots)
		self.max_uuv_vector = 7
		self.goal_dist = 50


	def calculate_nominal_paths(self, hotspot_dict, wpt_spacing):
		'''

		Inputs:
			hotspot_dict (Dict) :
			wpt_spacing (int) : distance (meters) between the nominal waypoints

		Returns:
			cc_paths (np.ndarray) : 
			all_paths (np.ndarray) : Nominal waypoints for traveling from hotspot to hotspot
									To access the path from hotspot 1 to 4 for example:
									all_paths[1][4]

		'''

		##For each of these hotspots, calculate the path to each other and the complete coverage algorithm to cover them
		cc_paths = []
		all_paths = []
		for a_idx in range(self.num_hotspots):
			## Get convex hull of each of the hotspots plus some buffer
			convexhull_a = sg.MultiPoint(hotspot_dict[a_idx][-1][:, 1:3]).convex_hull
			buffer_a = convexhull_a.buffer(5)
			ax, ay = buffer_a.exterior.coords.xy
			[minx, miny, maxx, maxy] = buffer_a.bounds
			cc_y_lines = np.arange(miny, maxy, 5)

			##Calculate the CC pattern on this hotspot
			cc_wpts = calc_mowing_lawn(buffer_a, cc_y_lines)
			cc_depths = self.Env.dfunc(cc_wpts)
			cc_paths.append(np.hstack((cc_wpts, cc_depths.reshape(-1,1))))

			hotspot_paths = []
			for b_idx in range(self.num_hotspots):
				if b_idx == a_idx:
					hotspot_paths.append([])
					continue

				convexhull_b = sg.MultiPoint(hotspot_dict[b_idx][-1][:, 1:3]).convex_hull
				buffer_b = convexhull_b.buffer(5)
				bx, by = buffer_b.exterior.coords.xy

				##Calculate the closest points between the hotspots
				pt1, pt2 = nearest_points(buffer_a, buffer_b)
				pt1_depth = self.Env.dfunc([pt1.x, pt1.y])[0]
				pt2_depth = self.Env.dfunc([pt2.x, pt2.y])[0]
				pt1_3d = np.array([pt1.x, pt1.y, pt1_depth])
				pt2_3d = np.array([pt2.x, pt2.y, pt2_depth])

				##NOMINAL PATH
				##Calculate the waypoints needed to traverse along the bottom of the seafloor
				##Split path into several points
				##Get the depth for all of those points
				##TODO? Smooth out the waypoints
				num_pts = abs(pt1.x - pt2.x)//wpt_spacing
				if num_pts == 0:
					path = np.array([pt1_3d, pt2_3d])
				else:
					waypts = np.array(zip(np.linspace(pt1.x, pt2.x, num_pts, endpoint=True), 
										  np.linspace(pt1.y, pt2.y, num_pts, endpoint=True)))
					waypts_depth = self.Env.dfunc(waypts)
					path = np.hstack((waypts, waypts_depth.reshape(-1,1) ))


				hotspot_paths.append(path)

			all_paths.append(hotspot_paths)

		return cc_paths, all_paths


	def follow_path_order(self, nominal_path, trash_dict):
		##TODO: Get follow_path_waypoints to work properly
		##Switch off between traveling to the cc areas and finding trash
		total_trip_energy_cost = 0
		total_trip_time_sec = 0
		uuv_path_state = 'path_following'
		# uuv_path_state = 'searching'
		for np_idx in range(len(nominal_path)):
			currently_following_path = nominal_path[np_idx]

			if uuv_path_state == 'path_following':
				energy_cost, time_cost_sec = follow_path_waypoints(currently_following_path, self.uuv)
				uuv_path_state = 'searching'

			if uuv_path_state == 'searching':
				energy_cost, time_cost_sec = search_for_trash(currently_following_path, trash_dict, self.uuv.pos)
				uuv_path_state = 'path_following'

			##TODO: Add up cost to travel this leg of the trip
			total_trip_energy_cost += energy_cost
			total_trip_time_sec += time_cost_sec

		return total_trip_energy_cost, total_trip_time_sec


	def main(self):
		## Create hotspots of trash and (optinal) visualize
		trash_x_centers = np.array([-250.98494701, -504.8406451, -132, 345, 876, 423]).reshape(-1,1)
		trash_y_centers = np.array([-508.96243035, -877.89326774, -687, 354, 120, 348]).reshape(-1,1)
		# trash_sigma = [[0.5, 0], [0, 0.1]] ##Can specify trash distribution covariance for all hotspots
		trash_sigma = [[], []]				 ##Use this for auto generated gaussian trash distributions
		hotspot_dict = init_hotspots(trash_x_centers, trash_y_centers, trash_sigma, 20, self.Env.dfunc)

		##Step the hotspot dict some x num times into the future
		xbound = [-self.Env.width/2, self.Env.width/2]
		ybound = [-self.Env.height/2, self.Env.height/2]
		map_dim = [xbound, ybound]

		fig = plt.figure()
		for xx in range(1000):
			hotspot_dict = update_trash_pos(hotspot_dict, 0, self.Env)
			# visualize_trash_step(hotspot_dict, True, map_dim) ##Uncomment to keep map view standardized
			visualize_trash_step(hotspot_dict)				##Use this for unstandardized map vie
			plt.pause(0.05)
		plt.show()

		##Calculate all complete coverage and inter-hotspot paths
		all_cc_paths, all_hotspot_paths = self.calculate_nominal_paths(hotspot_dict, 50)

		## Arbitrarily selected order of hotspot traversal
		## Get the path to follow
		hotspot_order = [0, 1, 2, 3, 4, 5]
		nominal_path = []
		for idx in range(len(hotspot_order)-1):
			nominal_path.extend([all_cc_paths[hotspot_order[idx]], 
								 all_hotspot_paths[hotspot_order[idx]][hotspot_order[idx+1]]])

		##Execute the path
		self.follow_path_order(nominal_path, hotspot_dict)




HS = Nom_Simulation()
HS.main()