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

from collections import OrderedDict
from numpy.linalg import norm

from trash_utils.haversine_dist import haversine
from trash_utils.finder_utils import (get_multi_current_block,
									  get_current_block,
									  get_depth_block,
									  grid_data,
									  find_bound_idx,
									  xy_to_lat_lon,
									  wpts_to_yaml)

from trash_utils.trash_lib import trash_2d_gauss

import pdb

'''
Deploy multiple trash hotspots in an environment

Calculate a path in 3D that would go from spot to spot

Do complete coverage in those spots

What angle should we optimize for in those hotspots if we were to arrive at that location?
'''

class Hotspot_Sampling():

	def __init__(self):
		self.place_bbox, self.ufunc, self.vfunc, self.dfunc = self.load_currents()
		self.width = haversine(self.place_bbox[0,0], self.place_bbox[0,1], self.place_bbox[1,0], self.place_bbox[1,1]) * 1000
		self.height = haversine(self.place_bbox[1,0], self.place_bbox[1,1], self.place_bbox[2,0], self.place_bbox[2,1]) * 1000
		self.u_boost = 30.5
		self.v_boost = 30.5
		self.num_hotspots = 6

		self.uuv_position = [0.0, 0.0, 0.0]
		self.max_uuv_vector = 7

		#For 4D path planning
		self.desired_speed = 2.5722 	##meters/second (5 knots)
		self.goal_dist = 75
		self.num_max_epochs = 2500
		self.E_appr = 75




	def load_currents(self):
		'''
		Initialize survey area bbox, u,v,depth functions

		Returns:
			place_bbox(np.ndarray):
			ufunc(function): time-dependent currents in u-direction. Inputs (time, depth, x,y)
			vfunc(function): time-dependent currents in v-direction. Inputs (time, depth, x,y)
			dfunc(function): returns bottom depth at that coordinate. Inputs (x,y)

		'''
		rospack = rospkg.RosPack()
		trash_finder_path = rospack.get_path("trash_finder")
		data_path = rospack.get_path("data_files")

		config = pickle.load(open(trash_finder_path + '/config/demo_configs.p', 'rb'))

		##hydrodynamic current
		current_path = data_path+'/ca_subCA_das_2020010615.nc'
		current_data = netcdf.NetCDFFile(current_path)

		currents1 = config['current_file1']
		currents_path1 = data_path + '/' + currents1
		currents2 = config['current_file2']
		currents_path2 = data_path + '/' + currents2
		depth = current_data.variables['depth'][:].copy()
		current_data.close()

		##bathymetry model
		topo = config['topo_file']
		topo_path = data_path + '/' + topo

		all_locations = pickle.load( open(trash_finder_path + "/config/locations.p", "rb"))
		place_bbox = all_locations["mission_bay_flatter_bbox"]

		ufunc, vfunc, uvfunc = get_multi_current_block(place_bbox, currents_path1, currents_path2)
		d_area, dfunc = get_depth_block(place_bbox, topo_path)

		return place_bbox, ufunc, vfunc, dfunc


	def init_hotspots(self, width, height):
		'''
		Create arbitrary 2D gaussians hotspots in a given area
		Dictionary of hotspots or an array of them?
		
		TODO: Are the centers okay?
		TODO: Is the sigma okay?
		TODO: Is the num_soda_cans okay?
		'''

		##Initialize 6 hotspots:
		# trash_x_centers = np.random.randint(-width/2, width/2, size=(self.num_hotspots,1))
		# trash_y_centers = np.random.randint(-height/2, height/2, size=(self.num_hotspots,1))
		
		trash_x_centers = np.array([-250.98494701, -504.8406451, -132, 345, 876, 423]).reshape(-1,1)
		trash_y_centers = np.array([-508.96243035, -877.89326774, -687, 354, 120, 348]).reshape(-1,1)

		sigma = [[0.5, 0.8], [0.8, 0.5]]
		num_soda_cans = 10
		trash_dict = {}

		for hotspot_idx in range(self.num_hotspots):
			hotspot_trash_coords = trash_2d_gauss([trash_x_centers[hotspot_idx][0], trash_y_centers[hotspot_idx][0]], sigma, num_soda_cans, self.dfunc)
			trash_dict[hotspot_idx] = [hotspot_trash_coords]

		return trash_dict


	def calc_mowing_lawn(self, bbox, y_bbox, start_end):
		'''
		Calculate the intersection of the lines with the bbox
		Orders the waypoints according to either left/right start/end

		Input:
			bbox (sh.Polygon) : some polygon to calculate the mowing lawn pattern
			y_bbox (np.ndarray) : Array of heights that the lines are calculated at
								Each point in this array corresponds to a line.
			start_end (str) : "right" or "left". Determines which side the first
							waypoint comes from. This determines the ordering of all
							the following waypoints.

		Output:
			waypoints (np.ndarray) : ordered waypoints on how to traverse this bbox
		'''
		all_intersections = []
		minx, miny, maxx, maxy = bbox.bounds
		for i in range(y_bbox.shape[0]):
			lines = sh.LineString([ [minx-abs(minx*0.4), y_bbox[i]],
									[maxx+abs(maxx*0.4), y_bbox[i]] ])
			intersections = bbox.intersection(lines)

			if intersections.is_empty == False:
				##TODO: sort the intersections so they are always left to right
				# print (zip(lines.xy[0], lines.xy[1]))
				# ziplines = zip(lines.xy[0], lines.xy[1])
				# npint = np.array(intersections)

				# plt.plot(bbox.exterior.coords.xy[0], bbox.exterior.coords.xy[1])
				# plt.plot(lines.xy[0], lines.xy[1])
				# if npint.ndim == 1:
				# 	plt.scatter(npint[0], npint[1])
				# else:
				# 	plt.scatter(npint[:,0], npint[:,1])
				# plt.show()
				# pdb.set_trace()

				all_intersections.append(np.array(intersections))

		##TODO: Should add in a check here to make sure there are intersecting lines

		##order the waypoints accordingly
		waypoints = []
		for i in range(len(all_intersections)):
			line_pts = all_intersections[i]

			if all_intersections[i].ndim == 1:
				waypoints.extend((line_pts, line_pts))
				continue

			if start_end == "right":
				if i%2 == 0:
					waypoints.extend((line_pts[1], line_pts[0]))
				else:
					waypoints.extend((line_pts[0], line_pts[1]))

			elif start_end == "left":
				if i%2 == 0:
					waypoints.extend((line_pts[0], line_pts[1]))
				else:
					waypoints.extend((line_pts[1], line_pts[0]))

		return np.array(waypoints)


	def simple_cc(self, search_bbox):
		rotation = np.arange(0, 360, 10)
		best_cost = 99999999999999999;
		for r in range(len(rotation)):
			rotated_bbox = sa.rotate(search_bbox, rotation[r]) ##Default, takes in rotation by degrees
			waypts = self.calc_mowing_lawn(rotated_bbox, y_bbox0, start_end="left")
			centered_waypoints = sa.rotate(sh.Polygon(waypts), -rotation[r])
			np_centered_waypoints = np.array(zip(centered_waypoints.exterior.coords.xy[0], 
									 centered_waypoints.exterior.coords.xy[1]))
			cost = self.est_propulsion(np_centered_waypoints, best_cost, 
											est_uuv_pos=False, use_bound=True, 
											visualize=False)

			if cost < best_cost:
				best_cost = cost
				best_waypoints = np_centered_waypoints
				best_rotation = rotation[r]
				# best_time = cost

		return best_cost


	def calc_heuristic_denom(self, xbound, ybound):
		'''
		Find the max current and calculate the heuristic denominator
		max || vc + vr ||



		'''

		d = grid_data(self.dfunc, xbound, ybound, 50, [], [])

		##Find the max current over a 4 day period (96 hrs total)
		min_u = 999999
		min_v = 999999
		# max_u = -999999
		# max_v = -999999
		for hrs in range(0, 96):
			u = grid_data(self.ufunc, xbound, ybound, 50, d, [hrs]) * self.u_boost
			v = grid_data(self.vfunc, xbound, ybound, 50, d, [hrs]) * self.v_boost
			min_u = min(min_u, np.min(u))
			min_v = min(min_v, np.min(v))
			# max_u = max(max_u, np.max(u))
			# max_v = max(max_v, np.max(v))

		##Calculate the desired heading and position
		goal_phi = np.deg2rad(45)
		goal_theta = np.deg2rad(45)
		mid_goal_x = self.desired_speed * sin(goal_phi) * cos(goal_theta)
		mid_goal_y = self.desired_speed * sin(goal_phi) * sin(goal_theta)
		mid_goal_z = self.desired_speed * cos(goal_phi)

		##Calculate the needed UUV offset and at what angle
		##uvv controls = desired_vector - ocean_vector
		desired_x_min = mid_goal_x - min_u
		desired_y_min = mid_goal_y - min_v
		# desired_x_max = mid_goal_x - max_u
		# desired_y_max = mid_goal_y - max_v
		desired_z = mid_goal_z
		uuv_vector_min = np.linalg.norm([desired_x_min, desired_y_min, desired_z])
		# uuv_vector_max = np.linalg.norm([desired_x_max, desired_y_max, desired_z])

		denom = uuv_vector_min + np.linalg.norm([min_u, min_v])
		# print("max total: ", uuv_vector_max + np.linalg.norm([max_u, max_v]))

		return denom


	def cost_to_waypoint(self, input_start_pos, goal_pos, goal_heading, time_now):
		'''
		Given headings and positions, calculate the cost of getting to that position
		Returns cost as timesteps and uuv_controls

		Input:
			start_pos (np.ndarray): (x,y,z)
			goal_pos (np.ndarray): (x,y,z)
			goal_heading (int, np.ndarray): radians [phi, theta]

		Returns:
			cost (float) : formula calculated from paper
			timesteps (int) : time it took to travel to the goal node in seconds
			controls (np.ndarray) : inputs to the system

		'''

		threshold2 = 2	##meters
		timesteps = 0

		uuv_controls = []
		# all_currents = []
		pos = np.copy(input_start_pos)

		goal_phi = goal_heading[0]
		goal_theta = goal_heading[1]

		# fig = plt.figure()
		# ax1 = fig.gca(projection='3d')
		# ax1.plot([input_start_pos[0]], [input_start_pos[1]], [input_start_pos[2]], 'bo')
		# ax1.plot([goal_pos[0]], [goal_pos[1]], [goal_pos[2]], 'bo')
		# ax1.plot([input_start_pos[0], goal_pos[0]], [input_start_pos[1], goal_pos[1]], [input_start_pos[2], goal_pos[2]], 'b--')
		epoch2 = 0
		max_num_epoch2 = 200
		while abs(np.linalg.norm([pos - goal_pos])) > threshold2 and epoch2 < max_num_epoch2:
			time_now_hrs = (time_now + timesteps)/3600.0
			##Calculate ocean u,v currents
			current_u = self.ufunc([time_now_hrs, abs(pos[2]), pos[0], pos[1]])[0] * self.u_boost
			current_v = self.vfunc([time_now_hrs, abs(pos[2]), pos[0], pos[1]])[0] * self.v_boost
			# current_vector = np.linalg.norm([current_u, current_v])
			# current_vector = [ocean_u, ocean_v, 0]

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

			# print ('mid_goal_x: ', mid_goal_x)
			# print ('mid_goal_y: ', mid_goal_y)
			# print ('mid_goal_z: ', mid_goal_z)

			##If mid_goal is outside the bounds of the map, then set to the edge
			if abs(pos[0]+mid_goal_x) > self.width/2:
				mid_goal_x = 0.0
				# print ("changed x")
			if abs(pos[1]+mid_goal_y) > self.height/2:
				mid_goal_y = 0.0
				# print ('changed y')
			if abs(pos[2]+mid_goal_z) > 60:
				mid_goal_z = 0.0
				# print ('changed z')

			# pdb.set_trace()

			##Calculate the needed UUV offset and at what angle
			##uvv controls = desired_vector - ocean_vector
			desired_x = mid_goal_x - current_u
			desired_y = mid_goal_y - current_v
			desired_z = mid_goal_z
			uuv_vector = np.linalg.norm([desired_x, desired_y, desired_z])
			# print ("uuv_vector: ", uuv_vector)

			# ##TODO: throw in check. If uuv_vector > possible result, return 9999999999 cost
			if uuv_vector > self.max_uuv_vector:
				print ("uuv_vector: ", uuv_vector)
				pdb.set_trace()
				return np.inf, np.inf, np.empty((0,6))

			uuv_phi = acos((desired_z / self.desired_speed))
			uuv_theta = atan2(desired_y, desired_x)

			# all_currents.append(current_vector)
			uuv_controls.append([uuv_vector, desired_x, desired_y, desired_z, uuv_phi, uuv_theta])
			timesteps += 1

			##update pos with resulting location
			pos += [mid_goal_x, mid_goal_y, mid_goal_z]
			# print ("dist to goal: ", abs(np.linalg.norm([pos - goal_pos])))

			epoch2 += 1

		# 	##TODO: visualize this makes sense
		# 	print ("start_pos: ", input_start_pos)
		# 	print ("goal_pos : ", goal_pos)
		# 	print ("pos: ", pos)
			# ax1.plot([pos[0]], [pos[1]], [pos[2]], 'ro')
		# 	print ("")

		# ax1.set_xlabel("x-axis")
		# ax1.set_ylabel("y-axis")
		# ax1.set_zlabel("depth")
		# plt.show()

		if not uuv_controls:
			time_now_hrs = (time_now + timesteps)/3600.0
			##Calculate ocean u,v currents
			current_u = self.ufunc([time_now_hrs, abs(pos[2]), pos[0], pos[1]])[0] * self.u_boost
			current_v = self.vfunc([time_now_hrs, abs(pos[2]), pos[0], pos[1]])[0] * self.v_boost
			# current_vector = np.linalg.norm([current_u, current_v])
			# current_vector = [ocean_u, ocean_v, 0]

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
			if abs(pos[2]+mid_goal_z) > 60:
				mid_goal_z = 0.0

			##Calculate the needed UUV offset and at what angle
			##uvv controls = desired_vector - ocean_vector
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

			# all_currents.append(current_vector)
			uuv_controls = [uuv_vector, desired_x, desired_y, desired_z, uuv_phi, uuv_theta]
			cost = (timesteps*(12.0/3600.0)) + (timesteps * ((uuv_controls[0]**3)**(0.333333))) * 0.30
		else:
			uuv_controls = np.array(uuv_controls)
			cost = (timesteps*(12.0/3600.0)) + (timesteps * ((uuv_controls[-1, 0]**3)**(0.333333))) * 0.30


		if epoch2 >= max_num_epoch2:
			return np.inf, np.inf, np.array(uuv_controls).reshape((-1, 6))
		else:
			return cost, timesteps, np.array(uuv_controls).reshape((-1, 6))
		# print ("cost: ", cost)
		# print ("timesteps (in seconds): ", timesteps)
		# print ("about to go to next point")
		# pdb.set_trace()

		# return cost, timesteps, np.array(uuv_controls).reshape((-1, 6))



	def find_optimal_path_nrmpc(self, time_start, epoch_max, start_pos, end_pos, heuristic_denom):
		'''
		Following the paper
		Predictive motion planning for AUVs subject to strong time-varying currents and forecasting uncertainties (Huynh, Van T., Dunbabin, Matthew, Smith, Ryan N.) (2015)

		They use a Nonlinear Robust Model Predictive Control (NRMPC) algorithm to define their system's state and control params. Then use A* to explore the space and control params until they get to their final destination.

		Input:
			time_start = time in seconds
			epoch_max = how many steps into the future to calculate
			start_pos = (x,y,depth)
			end_pos = (x,y,depth)
			heuristic_denom (float) = calculated value for heuristic denominator

		Returns:
			middle_pts = all the waypoints (x,y,depth) to get to the end_pos
			middle_time = all the times it would take to get to end_pos
			ending_control = ending speed in (u,v)

		'''
		##parameters?
		# E_appr = 10 ##Approximate, minimum energy per sampling time consumed by the vehicle at each time step
		# uuv_speed = 2.5722

		##Do sorted dictionary to hold the cost, heuristic_cost, and total_cost values
		visited_dict = {}  ##add in the new values once they've been calculated
		to_be_visited_dict = {} ##This will just be the sorted dict keys list

		##init starting node and its cost
		##heuristic_cost = euclidean_dist(current_pos, end_pos)/max(euclidean_dist(v_current + v_AUV_relative_speed)) * E_appr
		##heuristic cost is taken from the paper
		heuristic_cost = ( abs(np.linalg.norm([end_pos - start_pos]))/heuristic_denom )*self.E_appr

		to_be_visited_dict[tuple(start_pos)] = {'cost' : 0, 
										'heuristic_cost': heuristic_cost, 
										'total_cost': heuristic_cost, 
										'parent_pos': tuple([np.inf, np.inf, np.inf]), 
										'time_sec_at_this_node' : 0}

		##Find the possible positions around the goal
		##for each heading, calculate where the AUV would end up
		##also need to calculate in different depths
		base_h_angle = 45
		num_headings = 360/base_h_angle
		#num_phi_angles = (180/base_h_angle) - 1
		
		##Calculate the headings in spherical coordinate space
		theta = np.hstack((np.arange(0, 360, base_h_angle), np.arange(0, 360, base_h_angle), np.arange(0, 360, base_h_angle))).tolist()
		theta.extend([0.0, 0.0])
		theta = np.deg2rad(theta)

		##There is a smarter way to do this in an automated fashion:
		##Create a num_phi_angles x num_headings np.ones matrix
		##Multiply each row by np.arange(0, 180, base_h_angle)
		##Flatten and tolist()
		phi = np.hstack((np.ones((1,num_headings))*45, np.ones((1,num_headings))*90, np.ones((1,num_headings))*135))[0].tolist()
		phi.extend([0, 180])
		phi = np.deg2rad(phi)

		##Convert spherical coordinates into cartesian coordinates
		##Use these cartesian coordinates to figure out where to estimate where to plan for next
		x = np.sin(phi)*np.cos(theta)
		y = np.sin(phi)*np.sin(theta)
		z = np.cos(phi)

		all_hangles_cart = np.array(zip(x,y,z))
		all_hangles_sphere = np.array(zip(phi, theta))
		# all_timesteps = np.arange(0, 500, 8) ##TODO: Fix/figure this out
		current_pos = np.copy(start_pos)
		parent_cost = 0
		time_at_node = 0
		found_goal = False

		##Visualization
		fig = plt.figure()
		ax1 = fig.gca(projection='3d')
		#Plot the start and end points
		ax1.plot([start_pos[0]], [start_pos[1]], [start_pos[2]], 'ro')
		ax1.plot([end_pos[0]], [end_pos[1]], [end_pos[2]], 'bo')


		##While not within a threshold to the endpoint
		##calculate all of the headings
		threshold1 = 50 ##meters
		epoch = 0
		while abs(np.linalg.norm([current_pos - end_pos])) > threshold1 and epoch < epoch_max:
			# fig = plt.figure()
			# ax1 = fig.gca(projection='3d')
			# ax1.plot([current_pos[0]], [current_pos[1]], [current_pos[2]], 'go')

			##For each heading
			##Calculate the point 10km into the distance
			##Calculate the cost of getting to that point + parent cost = total cost
			##Calculate the heuristic of getting to that point
			##Save the point to the to_be_visted_list
			for angle_idx in range(all_hangles_cart.shape[0]):
				# print ("angle_idx: ", angle_idx)

				##Calculate the desired point 10m into the distance
				desired_heading_cart = all_hangles_cart[angle_idx]
				goal_x, goal_y, goal_z = desired_heading_cart*self.goal_dist + current_pos
				goal_pos = np.array([goal_x, goal_y, goal_z])

				if abs(goal_x) > self.width/2:
					goal_x = self.width/2 * np.sign(goal_x)
				if abs(goal_y) > self.height/2:
					goal_y = self.height/2 * np.sign(goal_y)
				if goal_z > 0:
					goal_z = 0
				if goal_z < -60:
					goal_z = -60
				goal_pos = np.array([goal_x, goal_y, goal_z])

				##If this point is within some distance to a previously calculated point, then move onto next step
				not_valid = False
				# print ("num keys: ", len(to_be_visited_dict.items()))
				for k,v in to_be_visited_dict.items():
					check_dist = abs(np.linalg.norm([goal_pos - np.array(k)]))

					if check_dist < self.goal_dist-5 and abs(np.linalg.norm(v['parent_pos'] - start_pos))>5:
						# print ("check dist: ", check_dist)
						# print ("too close")
						# print ("goal_pos: ", goal_pos)
						# print ("K    pos: ", np.array(k))
						not_valid = True
						# pdb.set_trace()
						break

				if not_valid:
					continue



				# print ("point is valid")


				##Calculate the cost of getting to that goal (cost is determined by formula from paper)
				# print ("current_pos: ", current_pos)
				# print ("goal_pos: ", goal_pos)

				# pdb.set_trace()
				desired_heading_sphere = all_hangles_sphere[angle_idx]
				cost, time_traveled, controls = self.cost_to_waypoint(current_pos, goal_pos, desired_heading_sphere, time_at_node/3600)
				self_and_parent_cost = parent_cost + cost
				# print ("self_and_parent_cost: ", self_and_parent_cost)

				heuristic_cost = ( abs(np.linalg.norm([end_pos - current_pos]))/heuristic_denom )*self.E_appr
				# print ('heuristic_cost: ', heuristic_cost)
				# print ("total_cost: ", heuristic_cost+self_and_parent_cost)

				# if tuple(goal_pos) in to_be_visited_dict:
				# 	print ("this key already exists in dict")
				# 	print ("goal_pos: ", goal_pos)
				# 	print ("current_pos: ", current_pos)
				# 	print (to_be_visited_dict[tuple(goal_pos)])
				# else:
				# 	print ("This key is not in the dict")


				##Append this cost to the unvisited list
				to_be_visited_dict[tuple(goal_pos)] = {'cost' : self_and_parent_cost, 
														'heuristic_cost': heuristic_cost, 
														'total_cost': heuristic_cost + self_and_parent_cost,
														'parent_pos': current_pos, 
														'time_sec_at_this_node' : time_at_node + time_traveled}
				
				# print ("added value to dict: ", goal_pos)
				# pdb.set_trace()

				##plot where the resulting point is and the cost of how much it takes to get to that point
				# ax1.plot([goal_x], [goal_y], [goal_z], 'bo')
				# ax1.plot([current_pos[0], goal_x], [current_pos[1], goal_y], [current_pos[2], goal_z], 'b--')
				# ax1.text(goal_x, goal_y, goal_z, str(heuristic_cost+self_and_parent_cost))


			# pdb.set_trace()

			##After we have calculated all these costs for all the headings, figure out which node to calculate from next
			##Sort the unvisited list by cost
			visited_dict[tuple(current_pos)] = to_be_visited_dict[tuple(current_pos)]
			del to_be_visited_dict[tuple(current_pos)]
			# print ("deleted something")
			sorted_cost_dict = sorted(to_be_visited_dict.items(), key=lambda x: (x[1]['total_cost']))

			##Pick the next node to visit
			lowest_cost_key = sorted_cost_dict[0][0]
			lowest_cost_items = sorted_cost_dict[0][1]
			# print ("next node to be visited: ", lowest_cost_key, lowest_cost_items)
			print ("dist to goal: ", abs(np.linalg.norm([end_pos - lowest_cost_key])))
			print ("lowest cost: ", lowest_cost_items['cost'])
			print ("lowest heuristic_cost: ", lowest_cost_items['heuristic_cost'])
			print ("lowest total_cost: ", lowest_cost_items['total_cost'])
			print ()
			# pdb.set_trace()

			ax1.plot([lowest_cost_key[0]], [lowest_cost_key[1]], [lowest_cost_key[2]], 'go')

			##Update the needed variables
			##Parent cost
			##Append current node to the visited list
			##Update timestart timestep values?
			
			parent_cost = lowest_cost_items['total_cost']
			time_at_node = lowest_cost_items['time_sec_at_this_node']
			current_pos = lowest_cost_key

			epoch += 1

			if abs(np.linalg.norm([current_pos - end_pos])) < threshold1:
				print ("WE ARE WITHIN REACH TO THE END POINT!")
				print ("WE ARE BREAKING LOOP")
				print ("Epochs: ", epoch)
				found_goal = True
				##backtrack through the nodes as a result
				break

		ax1.set_xlabel("x-axis")
		ax1.set_ylabel("y-axis")
		ax1.set_zlabel("depth")
		plt.show()

		if found_goal:
			print ("We found the end!!!!")
		else:
			print ("We did not reach the goal")

		pdb.set_trace()


	def main(self):
		## Get all the currents functions
		width = haversine(self.place_bbox[0,0], self.place_bbox[0,1], self.place_bbox[1,0], self.place_bbox[1,1]) * 1000
		height = haversine(self.place_bbox[1,0], self.place_bbox[1,1], self.place_bbox[2,0], self.place_bbox[2,1]) * 1000
		# xwidth = [-width/2, width/2]
		# yheight = [-height/2, height/2]
		# xy_dim = np.array([[-width/2, height/2], [width/2, height/2], [width/2, -height/2], [-width/2, -height/2]])

		## Create hotspots of trash
		hotspot_dict = self.init_hotspots(width, height)

		## Create a matrix containing cost to get from hotspot to hotspot
		## Create a hotspot grid, which contains the cost of how long it would take to do cc in the hotspot
		total_time_duration = 15*4*24*4
		cost_matrix = np.empty((total_time_duration, self.num_hotspots, self.num_hotspots))
		cc_cost_matrix = np.empty((total_time_duration, self.num_hotspots, 1))
		paths_matrix = np.empty((total_time_duration, self.num_hotspots, self.num_hotspots))

		## For every 15 minutes over 4 days:
		for ts in range(total_time_duration):
			convex_hotspots = []
			original_hotspots = []

			##For each hotspot
			for a_idx in range(self.num_hotspots-1):
				for b_idx in range(1, self.num_hotspots):
					## Get convex hull of each of the hotspots plus some buffer
					convexhull_a = sg.MultiPoint(hotspot_dict[a_idx][-1][:, 1:3]).convex_hull
					buffer_a = convexhull_a.buffer(5)
					ax, ay = buffer_a.exterior.coords.xy

					convexhull_b = sg.MultiPoint(hotspot_dict[b_idx][-1][:, 1:3]).convex_hull
					buffer_b = convexhull_b.buffer(5)
					bx, by = buffer_b.exterior.coords.xy

					original_a = hotspot_dict[a_idx][-1][:, 1:3]
					original_b = hotspot_dict[b_idx][-1][:, 1:3]

					convex_hotspots.append([ax,ay])
					original_hotspots.append(original_a)

					##Calculate the closest points to each hotspot
					pt1, pt2 = nearest_points(buffer_a, buffer_b)

					##Calculate the waypoints needed to traverse along the bottom of the seafloor
					##Split path into several points
					##Get the depth for all of those points
					num_x_pts = abs(pt1.x - pt2.x)//20
					# num_y_pts = abs(pt1.y - pt2.y)//20

					waypts = np.array(zip(np.linspace(pt1.x, pt2.x, num_x_pts), np.linspace(pt1.y, pt2.y, num_x_pts)))
					waypts_depth = self.dfunc(waypts)
					all_waypts = np.hstack((waypts, waypts_depth.reshape(-1,1) ))

					##TODO? Smooth out the waypoints
					##Save the paths waypoints. TODO: FIX
					# paths_matrix[ts, a_idx, b_idx] = np.hstack((waypts_depth, waypts))


					##Visualize/debugging in 2D
					# plt.plot(original_a[:,0], original_a[:,1], 'ro')
					# plt.plot(ax, ay, 'b')

					# plt.plot(original_b[:,0], original_b[:,1], 'ro')
					# plt.plot(bx, by, 'b')

					# plt.plot(pt1.x, pt1.y, 'g*')
					# plt.plot(pt2.x, pt2.y, 'g*')
					# plt.plot([pt1.x, pt2.x], [pt1.y, pt2.y], 'c--')

					# plt.plot(waypts[:,0], waypts[:,1], 'k*')

					# plt.gca().set_aspect('equal', adjustable='box')
					# plt.show()


					##Visualize 3D path in space
					##Set up 3D axis
					fig = plt.figure()
					ax1 = fig.gca(projection='3d')

					##Plot the hotspots in 2D
					ax_depth = self.dfunc(np.array(buffer_a.exterior.coords.xy).transpose())
					bx_depth = self.dfunc(np.array(buffer_b.exterior.coords.xy).transpose())
					ax1.plot(ax, ay, ax_depth, 'b')
					ax1.plot(bx, by, bx_depth, 'b')

					##Plot the 3D waypoints
					pt1_depth = self.dfunc([pt1.x, pt1.y])
					pt2_depth = self.dfunc([pt2.x, pt2.y])
					ax1.plot(waypts[:,0], waypts[:,1], waypts_depth, 'k*') #waypoints depth
					ax1.plot(waypts[:,0], waypts[:,1], waypts_depth, 'g--') #waypoints depth

					##FUTURE TODO: Throw in seafloor bottom?


					##Get the right angle to view it
					ax1.set_xlabel("x-axis")
					ax1.set_ylabel("y-axis")
					ax1.set_zlabel("depth")
					plt.show()

					print (all_waypts)
					##create yaml file with these waypoints
					wpts_to_yaml(all_waypts, '')

					pdb.set_trace()

					##Calculate the cost of getting from point to point (time for now)
					##Need to incorporate 3d distance to waypoint into path cost now



				convex_hotspots.append([bx,by])
				original_hotspots.append(original_b)

			## Calculate cost to get to each of the other hotspots
			## Convex hull the trash positions + add buffer
			## Calculate cost to survey the whole cluster of trash
			## Grab times to capture trash (we wouldn't know this initially though)
			## Save all the trash positions?


	def main_2d(self):
		## Get all the currents functions

		# xwidth = [-width/2, width/2]
		# yheight = [-height/2, height/2]
		# xy_dim = np.array([[-width/2, height/2], [width/2, height/2], [width/2, -height/2], [-width/2, -height/2]])

		##Get max current and calculate heuristic denominator
		xbound = [-self.width/2, self.width/2]
		ybound = [-self.height/2, self.height/2]
		heuristic_denom = self.calc_heuristic_denom(xbound, ybound)

		## Create hotspots of trash
		hotspot_dict = self.init_hotspots(self.width, self.height)

		## Create a matrix containing cost to get from hotspot to hotspot
		## Create a hotspot grid, which contains the cost of how long it would take to do cc in the hotspot
		total_time_duration = 15*4*24*4
		time_arr = np.arange(0, total_time_duration, 15)

		cost_matrix = np.empty((total_time_duration, self.num_hotspots, self.num_hotspots))
		cc_cost_matrix = np.empty((total_time_duration, self.num_hotspots, 1))
		paths_matrix = []

		## For every 15 minutes over 4 days:
		for ts in range(len(time_arr)):
			convex_hotspots = []
			original_hotspots = []

			##For each hotspot
			for a_idx in range(self.num_hotspots-1):
				## Get convex hull of each of the hotspots plus some buffer
				convexhull_a = sg.MultiPoint(hotspot_dict[a_idx][-1][:, 1:3]).convex_hull
				buffer_a = convexhull_a.buffer(5)
				ax, ay = buffer_a.exterior.coords.xy

				##TODO: Calculate how much time it would take to do cc on that hotspot



				for b_idx in range(1, self.num_hotspots):
					convexhull_b = sg.MultiPoint(hotspot_dict[b_idx][-1][:, 1:3]).convex_hull
					buffer_b = convexhull_b.buffer(5)
					bx, by = buffer_b.exterior.coords.xy

					original_a = hotspot_dict[a_idx][-1][:, 1:3]
					original_b = hotspot_dict[b_idx][-1][:, 1:3]

					convex_hotspots.append([ax,ay])
					original_hotspots.append(original_a)

					##Calculate the closest points to each hotspot
					pt1, pt2 = nearest_points(buffer_a, buffer_b)
					pt1_depth = self.dfunc([pt1.x, pt1.y])[0]
					pt2_depth = self.dfunc([pt2.x, pt2.y])[0]

					pt1_3d = np.array([pt1.x, pt1.y, pt1_depth])
					pt2_3d = np.array([pt2.x, pt2.y, pt2_depth])

					print ('pt1_3d: ', pt1_3d)
					print ('pt2_3d: ', pt2_3d)
					print ()

					##A* method
					self.find_optimal_path_nrmpc(0, self.num_max_epochs, pt1_3d, pt2_3d, heuristic_denom)
					print ("FINAL_OPTIMAL_PATH_NRMPC IS FINISHED TESTING")
					pdb.set_trace()

					###############STOP. CURRENTLY TESTING UP TO HERE

					##Calculate the waypoints needed to traverse along the bottom of the seafloor
					##NOMINAL PATH
					##Split path into several points
					##Get the depth for all of those points
					num_x_pts = abs(pt1.x - pt2.x)//20
					# num_y_pts = abs(pt1.y - pt2.y)//20

					waypts = np.array(zip(np.linspace(pt1.x, pt2.x, num_x_pts), np.linspace(pt1.y, pt2.y, num_x_pts)))
					waypts_depth = self.dfunc(waypts)
					all_waypts = np.hstack((waypts, waypts_depth.reshape(-1,1) ))

					##TODO? Smooth out the waypoints
					##Save the paths waypoints. TODO: FIX
					# paths_matrix[ts, a_idx, b_idx] = np.hstack((waypts_depth, waypts))


					#Visualize/debugging in 2D
					plt.plot(original_a[:,0], original_a[:,1], 'ro')
					plt.plot(ax, ay, 'b')

					plt.plot(original_b[:,0], original_b[:,1], 'ro')
					plt.plot(bx, by, 'b')

					plt.plot(pt1.x, pt1.y, 'g*')
					plt.plot(pt2.x, pt2.y, 'g*')
					plt.plot([pt1.x, pt2.x], [pt1.y, pt2.y], 'c--')

					plt.plot(waypts[:,0], waypts[:,1], 'k*')

					plt.gca().set_aspect('equal', adjustable='box')
					plt.show()

					pdb.set_trace()


					# ##Visualize 3D path in space
					# ##Set up 3D axis
					# fig = plt.figure()
					# ax1 = fig.gca(projection='3d')

					# ##Plot the hotspots in 2D
					# ax_depth = self.dfunc(np.array(buffer_a.exterior.coords.xy).transpose())
					# bx_depth = self.dfunc(np.array(buffer_b.exterior.coords.xy).transpose())
					# ax1.plot(ax, ay, ax_depth, 'b')
					# ax1.plot(bx, by, bx_depth, 'b')

					# ##Plot the 3D waypoints
					# pt1_depth = self.dfunc([pt1.x, pt1.y])
					# pt2_depth = self.dfunc([pt2.x, pt2.y])
					# ax1.plot(waypts[:,0], waypts[:,1], waypts_depth, 'k*') #waypoints depth
					# ax1.plot(waypts[:,0], waypts[:,1], waypts_depth, 'g--') #waypoints depth

					# ##FUTURE TODO: Throw in seafloor bottom?
					

					# ##Get the right angle to view it
					# ax1.set_xlabel("x-axis")
					# ax1.set_ylabel("y-axis")
					# ax1.set_zlabel("depth")
					# plt.show()

					# print (all_waypts)
					# ##create yaml file with these waypoints
					# wpts_to_yaml(all_waypts, '')

					# pdb.set_trace()

					##Calculate the cost of getting from point to point (time for now)
					##Need to incorporate 3d distance to waypoint into path cost now



				convex_hotspots.append([bx,by])
				original_hotspots.append(original_b)

			## Calculate cost to get to each of the other hotspots
			## Convex hull the trash positions + add buffer
			## Calculate cost to survey the whole cluster of trash
			## Grab times to capture trash (we wouldn't know this initially though)
			## Save all the trash positions?



HS = Hotspot_Sampling()
HS.main_2d()