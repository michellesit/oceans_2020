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
		self.u_boost = 1.5
		self.v_boost = 1.5
		self.num_hotspots = 6

		self.uuv_position = [0.0, 0.0, 0.0]

		#For 4D path planning
		self.global_timesteps = 0
		self.desired_speed = 2.5722 	##meters/second (5 knots)
		self.max_uuv_vector = 7			##TODO: check this is okay


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
		trash_x_centers = np.random.randint(-width/2, width/2, size=(self.num_hotspots,1))
		trash_y_centers = np.random.randint(-height/2, height/2, size=(self.num_hotspots,1))
		
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



	def cost_to_waypoint(start_pos, goal_pos, goal_heading, time_now):
		'''
		Given headings and positions, calculate the cost of getting to that position
		Returns cost as timesteps and uuv_controls

		Input:
			start_pos (np.ndarray): (x,y,z)
			goal_pos (np.ndarray): (x,y,z)
			goal_heading (int, np.ndarray): radians [phi, theta]

		TEST

		'''

		threshold = 5	##meters
		timesteps = 0

		uuv_controls = []
		# all_currents = []
		pos = start_pos
		while abs(np.linalg.norm(pos - goal_pos)) > threshold:
			##Calculate ocean u,v currents
			current_u = self.ufunc([time_now + timesteps, pos[2], pos[0], pos[1]])
			current_v = self.vfunc([time_now + timesteps, pos[2], pos[0], pos[1]])
			# current_vector = np.linalg.norm([current_u, current_v])
			# current_vector = [ocean_u, ocean_v, 0]

			##Calculate the desired heading and position
			goal_phi = goal_heading[0]
			goal_theta = goal_heading[1]
			mid_goal_x = self.desired_speed * cos(goal_phi) * cos(goal_theta)
			mid_goal_y = self.desired_speed * sin(goal_phi) * sin(goal_phi)
			mid_goal_z = self.desired_speed * cos(goal_phi)

			##Calculate the needed UUV offset and at what angle
			##uvv controls = desired_vector - ocean_vector
			desired_x = mid_goal_x - current_u
			desired_y = mid_goal_y - current_v
			desired_z = mid_goal_z
			uuv_vector = np.linalg.norm([desired_x, desired_y, desired_z])

			##TODO: throw in check. If uuv_vector > possible result, return 9999999999 cost
			if uuv_vector > self.max_uuv_vector:
				return 99999999999999999, 99999999999999999999, np.empty((0,6))

			uuv_phi = acos((desired_z / self.desired_speed))
			uuv_theta = acos((desired_x/(self.desired_speed * sin(uuv_phi))))

			# all_currents.append(current_vector)
			uuv_controls.append([uuv_vector, desired_x, desired_y, desired_z, uuv_phi, uuv_theta])
			timesteps += 1

			##update pos with resulting location
			pos = [mid_goal_x, mid_goal_y, mid_goal_z]

			##TODO: visualize this makes sense



		cost = (timesteps*(12/3600)) + (timesteps * ((uuv_controls[-1, 0]**3)**(1/3)) * 0.5)
		print ("cost: ", cost)

		return cost, timesteps, np.array(uuv_controls).reshape((-1, 6))



	def find_optimal_path_nrmpc(time_start, epoch, start_pos, end_pos, heuristic_denom):
		'''
		Following the paper
		Predictive motion planning for AUVs subject to strong time-varying currents and forecasting uncertainties (Huynh, Van T., Dunbabin, Matthew, Smith, Ryan N.) (2015)

		They use a Nonlinear Robust Model Predictive Control (NRMPC) algorithm to define their system's state and control params. Then use A* to explore the space and control params until they get to their final destination.

		Input:
			time_start = time in seconds
			epoch = how many steps into the future to calculate
			start_pos = (x,y,depth)
			end_pos = (x,y,depth)
			heuristic_denom (float) = calculated value for heuristic denominator

		Returns:
			middle_pts = all the waypoints (x,y,depth) to get to the end_pos
			middle_time = all the times it would take to get to end_pos
			ending_control = ending speed in (u,v)

		'''
		##parameters?
		E_appr = 5 ##Approximate, minimum energy per sampling time consumed by the vehicle at each time step
		uuv_speed = 2.5722

		##Do sorted dictionary to hold the cost, heuristic_cost, and total_cost values
		all_map = {}

		##init starting node and its cost
		##heuristic_cost = euclidean_dist(current_pos, end_pos)/max(euclidean_dist(v_current + v_AUV_relative_speed)) * E_appr
		##heuristic cost is taken from the paper
		heuristic_cost = ( norm(start_pos, end_pos)/heuristic_denom )*E_appr
		all_map[tuple(start_pos)] = {'cost' : 0, 'heuristic_cost': heuristic_cost, 'total_cost': heuristic_cost}

		visited_list = []
		to_be_visited_list = {} ##This will just be the sorted dict keys list

		##Find the possible positions around the goal
		##for each heading, calculate where the AUV would end up
		##also need to calculate in different depths
		base_h_angle = 45
		num_headings = 360/base_h_angle
		
		##Calculate the headings in spherical coordinate space
		theta = np.hstack((np.arange(0, 360, base_h_angle), np.arange(0, 360, base_h_angle), np.arange(0, 360, base_h_angle))).tolist()
		theta.extend([0.0, 0.0])
		theta = np.deg2rad(theta)

		phi = np.hstack((np.ones((1,8))*45, np.ones((1,8))*90, np.ones((1,8))*135))[0].tolist()
		phi.extend([0, 180])
		phi = np.deg2rad(phi)

		##Convert spherical coordinates into cartesian coordinates
		##Use these cartesian coordinates to figure out where to estimate where to plan for next
		x = np.sin(phi)*np.cos(theta)
		y = np.sin(phi)*np.sin(theta)
		z = np.cos(phi)


		all_hangles = np.array(zip(x,y,z))
		all_timesteps = np.arange(0, 500, 8) ##TODO: Fix/figure this out
		current_pos = start_pos

		##Calculate for each heading
		for angle_idx in range(all_hangles.shape[0]):
			##Calculate the desired point into the distance
			desired_heading = all_hangles[angle_idx]
			goal_x, goal_y, goal_z = desired_heading*10 + current_pos
			current_time_hrs = (time_start + all_timesteps[angle_idx])/3600 ##TODO Does this make sense?

			##Calculate the cost of getting to that goal
			cost, time_traveled, controls = cost_to_waypoint(current_pos, goal_pos, goal_heading, current_time_hrs)

			print ('cost: ', cost)

			heuristic_cost = ( norm(current_pos, end_pos)/heuristic_denom )*E_appr
			print ('heuristic_cost: ', heuristic_cost)

			##Append this cost to the unvisited list
			##TODO: how do we build the graph so that we can trace the min cost path through the whole thing?


		##Sort the unvisited list by cost


		##Pick the next node to visit




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


	def shortest_dist_sim(self, total_time_duration):
		pass


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
			u = grid_data(self.ufunc, xbound, ybound, 50, d, [hrs])
			v = grid_data(self.vfunc, xbound, ybound, 50, d, [hrs])
			min_u = min(min_u, np.min(u))
			min_v = min(min_v, np.min(v))
			# max_u = max(max_u, np.max(u))
			# max_v = max(max_v, np.max(v))

		##Calculate the desired heading and position
		goal_phi = np.deg2rad(45)
		goal_theta = np.deg2rad(45)
		mid_goal_x = self.desired_speed * cos(goal_phi) * cos(goal_theta)
		mid_goal_y = self.desired_speed * sin(goal_phi) * sin(goal_phi)
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
		width = haversine(self.place_bbox[0,0], self.place_bbox[0,1], self.place_bbox[1,0], self.place_bbox[1,1]) * 1000
		height = haversine(self.place_bbox[1,0], self.place_bbox[1,1], self.place_bbox[2,0], self.place_bbox[2,1]) * 1000
		# xwidth = [-width/2, width/2]
		# yheight = [-height/2, height/2]
		# xy_dim = np.array([[-width/2, height/2], [width/2, height/2], [width/2, -height/2], [-width/2, -height/2]])

		##Get max current and calculate heuristic denominator
		xbound = [-width/2, width/2]
		ybound = [-height/2, height/2]
		heuristic_denom = self.calc_heuristic_denom(xbound, ybound)

		## Create hotspots of trash
		hotspot_dict = self.init_hotspots(width, height)
		print (hotspot_dict)
		pdb.set_trace()

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


					##A* method
					find_optimal_path_nrmpc(timenow[ts], 100, pt1, pt2)


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