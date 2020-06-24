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
									  wpts_to_yaml,
									  load_currents,
									  get_width,
									  get_height)

from trash_utils.trash_lib import trash_2d_gauss, init_hotspots, visualize_trash_flow, update_trash_pos,visualize_trash_step

import pdb

'''
Deploy multiple trash hotspots in an environment

Calculate a path in 3D that would go from spot to spot

Do complete coverage in those spots
'''

class Hotspot_Sampling():

	def __init__(self):
		self.place_bbox, self.ufunc, self.vfunc, self.dfunc = load_currents()
		self.width = get_width(self.place_bbox)
		self.height = get_height(self.place_bbox)
		self.u_boost = 1
		self.v_boost = 1

		# self.u_boost = 30.5
		# self.v_boost = 30.5
		# self.u_boost = 50.5
		# self.v_boost = 50.5
		self.num_hotspots = 6

		self.uuv_position = [0.0, 0.0, 0.0]
		self.max_uuv_vector = 7
		self.max_depth = 50

		#For 4D path planning
		self.desired_speed = 2.5722 	##meters/second (5 knots)
		self.goal_dist = 50



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


	def cost_to_waypoint(self, input_start_pos, goal_pos, goal_heading, time_now, vis_ctw=False):
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



	def main(self):
		# ##Get max current and calculate heuristic denominator
		xbound = [-self.width/2, self.width/2]
		ybound = [-self.height/2, self.height/2]

		## Create hotspots of trash
		trash_x_centers = np.array([-250.98494701, -504.8406451, -132, 345, 876, 423]).reshape(-1,1)
		trash_y_centers = np.array([-508.96243035, -877.89326774, -687, 354, 120, 348]).reshape(-1,1)
		# trash_sigma = [[0.5, 0], [0, 0.1]]
		trash_sigma = [[], []]
		hotspot_dict = init_hotspots(trash_x_centers, trash_y_centers, trash_sigma, 20, self.dfunc)

		map_dim = [xbound, ybound]

		##step the trash dict 1 timestep
		fig = plt.figure()
		for xx in range(100):
			hotspot_dict = update_trash_pos(hotspot_dict, 0, self.ufunc, self.vfunc, self.dfunc, self.width, self.height)

			# visualize_trash_step(hotspot_dict, True, map_dim)
			visualize_trash_step(hotspot_dict)
			plt.pause(0.05)
			plt.clf()

		plt.show()

		pdb.set_trace()

		## Create a matrix containing cost to get from hotspot to hotspot
		## Create a hotspot grid, which contains the cost of how long it would take to do cc in the hotspot
		total_time_duration = 15*4*24*4*60
		time_arr = np.arange(0, total_time_duration, 3600*3)

		cost_matrix = np.empty((total_time_duration, self.num_hotspots, self.num_hotspots))
		nom_cost_matrix = np.empty((total_time_duration, self.num_hotspots, self.num_hotspots))
		cc_cost_matrix = np.empty((total_time_duration, self.num_hotspots, 1))
		paths_matrix = []

		## For every 15 minutes over 4 days:
		for ts in range(len(time_arr)):
			convex_hotspots = []
			original_hotspots = []

			##For each hotspot
			for a_idx in range(self.num_hotspots):
				## Get convex hull of each of the hotspots plus some buffer
				convexhull_a = sg.MultiPoint(hotspot_dict[a_idx][-1][:, 1:3]).convex_hull
				buffer_a = convexhull_a.buffer(5)
				ax, ay = buffer_a.exterior.coords.xy

				##TODO: Calculate how much time it would take to do cc on that hotspot



				for b_idx in range(self.num_hotspots):
					if b_idx == a_idx:
						print ("ts: ", ts, a_idx, b_idx)
						print ("a_idx and b_idx are the same. Passing!")
						continue

					print ("ts: ", ts, a_idx, b_idx)
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

					# ##A* method
					# astar_path, astar_cost, astar_time_sec = self.find_optimal_path_nrmpc(time_arr[ts], self.num_max_epochs, pt1_3d, pt2_3d, heuristic_denom)

					# acompcost = 0
					# acomptimecost = 0
					# for apt_idx in range(astar_path.shape[0]-1):
					# 	astar_diff = astar_path[apt_idx+1,:] - astar_path[apt_idx, :]
					# 	astar_h = np.linalg.norm(astar_diff)
					# 	astar_phi = acos(astar_diff[2] / astar_h)
					# 	astar_theta = atan2(astar_diff[1], astar_diff[0])
					# 	astar_heading = [astar_phi, astar_theta]
					# 	acost, atime, actrl = self.cost_to_waypoint(astar_path[apt_idx,:], astar_path[apt_idx+1,:], astar_heading, time_arr[ts])
						
					# 	acompcost += np.sum(actrl[:,0])
					# 	acomptimecost += atime

					# print ("ACOST? ", acompcost)
					# print ("ATIME? ", acomptimecost)

					# cost_matrix[ts][a_idx][b_idx] = astar_cost
					# print ("astar_cost: ", astar_cost)
					# print ("FINAL_OPTIMAL_PATH_NRMPC IS FINISHED TESTING")

					# pdb.set_trace()

					#Calculate the waypoints needed to traverse along the bottom of the seafloor
					##NOMINAL PATH
					##Split path into several points
					##Get the depth for all of those points
					num_x_pts = abs(pt1.x - pt2.x)//50
					# num_y_pts = abs(pt1.y - pt2.y)//20

					waypts = np.array(zip(np.linspace(pt1.x, pt2.x, num_x_pts, endpoint=True), np.linspace(pt1.y, pt2.y, num_x_pts, endpoint=True)))
					waypts_depth = self.dfunc(waypts)
					all_waypts = np.hstack((waypts, waypts_depth.reshape(-1,1) ))

					##TODO? Smooth out the waypoints
					##Save the paths waypoints. TODO: FIX
					# paths_matrix[ts, a_idx, b_idx] = np.hstack((waypts_depth, waypts))

					##TODO: Calculate the cost of traveling this path
					print ("all_waypts: ", all_waypts)
					# print ("pt1_3d: ", pt1_3d)
					# print ("pt2_3d: ", pt2_3d)

					nom_total_cost = 0
					nom_total_timecost = 0
					for nom_pt_idx in range(all_waypts.shape[0]-1):
						nominal_diff = all_waypts[nom_pt_idx+1] - all_waypts[nom_pt_idx]
						nominal_h = np.linalg.norm(nominal_diff)
						nominal_phi = acos(nominal_diff[2] / nominal_h)
						nominal_theta = atan2(nominal_diff[1], nominal_diff[0])
						nominal_heading = [nominal_phi, nominal_theta]
						nom_alg_cost, nom_timesteps, nom_controls = self.cost_to_waypoint(all_waypts[nom_pt_idx, :], all_waypts[nom_pt_idx+1, :], nominal_heading, time_arr[ts])

						nom_total_cost += np.sum(nom_controls[:, 0])
						nom_total_timecost += nom_timesteps

					# print ("astar_cost: ", astar_cost)
					# print ("final nom_total_cost: ", nom_total_cost)
					# print ("difference between costs: ", astar_cost - nom_total_cost)
					nom_cost_matrix[ts][a_idx][b_idx] = nom_total_cost
					# pdb.set_trace()

					# #Visualize/debugging in 2D
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

					# pdb.set_trace()

					##STRAIGHT LINE PATH
					str_waypts_depth = [self.dfunc([pt1.x, pt1.y])[0], self.dfunc([pt2.x, pt2.y])[0]]
					all_str_waypts = np.array(zip(np.linspace(pt1.x, pt2.x, num_x_pts, endpoint=True), np.linspace(pt1.y, pt2.y, num_x_pts, endpoint=True), np.linspace(str_waypts_depth[0], str_waypts_depth[1], num_x_pts, endpoint=True)))
					str_total_cost = 0
					str_total_timecost = 0
					for str_pt_idx in range(all_str_waypts.shape[0]-1):
						str_diff = all_str_waypts[str_pt_idx+1] - all_str_waypts[str_pt_idx]
						str_h = np.linalg.norm(str_diff)
						str_phi = acos(str_diff[2] / str_h)
						str_theta = atan2(str_diff[1], str_diff[0])
						str_heading = [str_phi, str_theta]
						str_alg_cost, str_timecost, str_controls = self.cost_to_waypoint(all_str_waypts[str_pt_idx, :], all_str_waypts[str_pt_idx+1, :], str_heading, time_arr[ts])

						str_total_timecost += str_timecost
						str_total_cost += np.sum(str_controls[:, 0])

					print ("ts: ", ts, a_idx, b_idx)
					print ("astar_cost prev: ", astar_cost)
					print ("astar_cost: ", acompcost)
					print ("final nom_total_cost: ", nom_total_cost)
					print ("difference between costs: ", acompcost - nom_total_cost)
					print ("final str_total_cost: ", str_total_cost)
					print ("difference between costs: ", acompcost - str_total_cost)
					# print ("astar_time_cost: ", astar_time_sec)
					print ("astar_time_cost: ", acomptimecost)
					print ("final_nom_total_timecost: ", nom_total_timecost)
					print ("final_nom_time_cost: ", str_total_timecost)

					# pdb.set_trace()

					# #Visualize/debugging in 2D
					# plt.plot(original_a[:,0], original_a[:,1], 'ro')
					# plt.plot(ax, ay, 'b')

					# plt.plot(original_b[:,0], original_b[:,1], 'ro')
					# plt.plot(bx, by, 'b')

					# plt.plot(pt1.x, pt1.y, 'g*')
					# plt.plot(pt2.x, pt2.y, 'g*')
					# plt.plot([pt1.x, pt2.x], [pt1.y, pt2.y], 'c--')

					# plt.plot(all_str_waypts[:,0], str_waypts[:,1], 'k*')

					# plt.gca().set_aspect('equal', adjustable='box')
					# plt.show()

					# pdb.set_trace()

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

			# pdb.set_trace()

			## Calculate cost to get to each of the other hotspots
			## Convex hull the trash positions + add buffer
			## Calculate cost to survey the whole cluster of trash
			## Grab times to capture trash (we wouldn't know this initially though)
			## Save all the trash positions?

		pdb.set_trace()
		'''

HS = Hotspot_Sampling()
HS.main()