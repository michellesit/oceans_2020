import pickle
from math import atan2, sin, cos

import rospkg
from scipy.io import netcdf
import numpy as np
import shapely.geometry as sh
import shapely.affinity as sa
import matplotlib.pyplot as plt

from trash_utils.haversine_dist import haversine
from trash_utils.finder_utils import (get_multi_current_block,
									  get_current_block,
									  get_depth_block,
									  grid_data,
									  find_bound_idx,
									  xy_to_lat_lon)

from trash_utils.visualize import visualize_area

import pdb

'''
Goal: discretizes the map into a lawn mower pattern in the direction of interest
	Takes the shortest dimension (which means the rows will be long. ie less turns)
	Calculates the waypoints needed to complete the pattern

TODO: incorporate contours/gradient difference into global decomposition
TODO: Incorporte uuv's last known position when calculating the most optimal route
'''

##Do we need to calculate the gradient of the lines?
##For now, assume the given bbox is rectangular
class Mowing_Lawn_BBox_Search:

	def __init__(self, bbox):
		##Decomposition parameters
		self.global_bbox_height_max = 150  ##meters
		self.global_bbox_width_max = 150   ##meters

		##Check these measurements are valid
		self.search_radius = 5			##meters
		self.search_width = 5			##meters
		self.rotation_deg = 10			##degrees


		##uuv propulsion stats
		self.goal_threshold = 5			##meters
		self.uuv_heading = 0			##radians
		self.uuv_speed = 2.5722 		##meters/second
		self.uuv_position = bbox[3,:]

		self.global_timesteps = 0			##seconds since beginning

		self.global_bbox = sh.Polygon(bbox)


	def global_decomp(self, visualize=False):
		'''
		Breaks down the global bbox into smaller more managable boxes

		Input:
			visualize (bool) : Set True to see global current map with
							   all smaller boxes overlayed

		Returns:
			all_bbox (list) : List of sh.Polygon (dim = 5 x 2) which are the
				coordinates of the local bbox shapes
			np_all_bbox (np.ndarray) : numpy friendly format of all_bbox ((x-1)*(y-1) x 2)
		'''
		x_bbox, y_bbox = self.calc_bbox_lines(self.global_bbox, 
											  self.global_bbox_height_max,
											  self.global_bbox_width_max)

		##TODO: This is time consuming for many points. Get rid of the two for loops
		all_bbox = []
		np_all_bbox = []
		for i in range(x_bbox.shape[0]-1):
			for j in range(y_bbox.shape[0]-1):
				bbox = sh.Polygon([ [x_bbox[i], y_bbox[j]], 
									[x_bbox[i+1], y_bbox[j]],
									[x_bbox[i+1], y_bbox[j+1]],
									[x_bbox[i], y_bbox[j+1]],
									[x_bbox[i], y_bbox[j]] ])
				all_bbox.append(bbox)

				np_all_bbox.append([[x_bbox[i], y_bbox[j]],
									[x_bbox[i+1], y_bbox[j]],
									[x_bbox[i+1], y_bbox[j+1]],
									[x_bbox[i], y_bbox[j+1]],
									[x_bbox[i], y_bbox[j]] ])

		np_all_bbox = np.array(np_all_bbox)
		
		##Visualize all the bbox here:
		if visualize:
			visualize_area(place_bbox, current_path, topo_path)
			reshaped = np_all_bbox.reshape((-1,2))
			plt.plot(place_bbox[:,0], place_bbox[:,1])
			plt.plot(reshaped[:,0], reshaped[:,1], color="black", linewidth=3)
			plt.show()

		return all_bbox, np_all_bbox


	def calc_bbox_lines(self, bbox, height_radius, width_radius):
		'''
		Calculates the number of lines that will cross the bbox
			based on the spacing values given

		Input:
			bbox (Polygon) 		: Polygon([upper left, upper right, bottom right, 
										   bottom left coords, upper left])
			height_radius (int) : height of the search rows
			width_radius (int)  : width of the search columns

		Returns:
			x_bbox (np.ndarray) : array of the x values for the smaller boxes
			y_bbox (np.ndarray) : array of the y values for the smaller boxes

		Make sure the boxes are odd rows with odd number of boxes?

		'''
		minx, miny, maxx, maxy = bbox.bounds
		bbox_height = abs(maxy - miny)
		bbox_width = abs(maxx - minx)

		num_height_lines = bbox_height//height_radius
		num_width_lines = bbox_width//width_radius

		if num_width_lines <= 0:
			x_bbox = np.array([minx, maxx])
		else:
			x_bbox = np.linspace(minx, maxx, (num_width_lines)+2)

		if num_height_lines <= 0:
			y_bbox = np.array([miny, maxy])
		else:
			y_bbox = np.linspace(miny, maxy, (num_height_lines)+2)

		return x_bbox, y_bbox


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


	def est_propulsion(self, waypoints, bound_cost, est_uuv_pos=True, use_bound=True, visualize=False):
		'''
		For each waypoint in the path, iteratively calculate how long it would take to get there.
		Do some vector math to calculate what direction/where you'd end up based 
			on currents and the uuv's vector heading/power

		Visualiztion shows an animation of the UUV's progress moving toward each waypoint

		Calculates for each 1 second (since we are traveling at 5knots = 2.5722m/sec)
		uuv_speed = constant for now

		Inputs:
			waypoints (np.ndarray) : Ordered waypoints that the UUV needs to take.
			bound_cost (int) : Branch and bound optimization. The max cost that this method may take.
			est_uuv_pos (bool) : True = Sets uuv position to be the first waypoint in the given waypoints.
								False, uses the last known position of the uuv (self.uuv_position)
			use_bound (bool) : Branch and bound optimization. If the timestep value is greater than
								bound_cost, then the method immediately terminates and returns an
								unreasonable cost.
			visualize (bool) : Shows an animation of the UUV progressing toward each waypoint at each step
		Returns:
			timesteps (int) : number of timesteps it took to traverse all these waypoints

		'''

		##Set uuv position to be the first waypoint
		##Otherwise, use the uuv's current position as its starting location
		if est_uuv_pos:
			self.uuv_position = waypoints[1]

		##Visualizes the path that will be taken
		if visualize:
			plt.subplots(111)
			ax = plt.subplot(111)
			plt.plot(waypoints[:,0], waypoints[:,1])
		
		timesteps = 0
		for wp in range(len(waypoints)):
			goal = waypoints[wp]

			while abs(np.linalg.norm(self.uuv_position - goal)) > self.goal_threshold:
				##Calculate the heading to that location
				desired_heading = atan2(goal[1] - self.uuv_position[1], goal[0] - self.uuv_position[0])
				goal_u = cos(desired_heading) * np.linalg.norm(goal - self.uuv_position)
				goal_v = sin(desired_heading) * np.linalg.norm(goal - self.uuv_position)
				current_time_hrs = (timesteps + self.global_timesteps)/3600

				z = abs(dfunc(self.uuv_position))
				current_u = ufunc([current_time_hrs, z, self.uuv_position[0], self.uuv_position[1]])[0]
				current_v = vfunc([current_time_hrs, z, self.uuv_position[0], self.uuv_position[1]])[0]
				# current_heading = atan2(current_v, current_u)

				##vector math
				desired_u = goal_u + current_u
				desired_v = goal_v - current_v
				resulting_heading = atan2(desired_v, desired_u)

				uuv_u = cos(resulting_heading) * self.uuv_speed
				uuv_v = sin(resulting_heading) * self.uuv_speed

				resulting_speed = np.array([uuv_u + current_u, uuv_v + current_v])

				##TODO: What does each Circle represent?
				##TODO: What do each arrow represents?
				if visualize:
					ax.quiver(self.uuv_position[0], self.uuv_position[1], uuv_u, uuv_v, scale=1, scale_units='xy', color='green') ##uuv
					wp_c = plt.Circle((goal[0], goal[1]), 1, color='black') ##waypoint
					ax.add_artist(wp_c)
					uuv_end = plt.Circle((self.uuv_position[0] + resulting_speed[0], self.uuv_position[1] + resulting_speed[1]), 1, color="blue")
					ax.add_artist(uuv_end)
					plt.pause(0.05)

				self.uuv_position += resulting_speed
				self.uuv_heading = resulting_heading

				##add to timestep count (time)
				timesteps += 1
				if use_bound and timesteps > bound_cost:
					return 99999999999999999

		return timesteps


	def search(self, visualize=False):
		'''
		Calculate waypoints for each of the smaller bbox areas
		for each bbox in all_bbox:
			Rotate by some amount
			Calculate the waypoints for each
			Calculate the cost of traveling that path
			Keep the best cost and rotation

		TODO: comments

		Input:
			visualize (bool) : 
		Returns: 
			global_waypoints (Dict) : The lowest cost waypoints
			global_cost (list) : The best cost from each local bbox. Sum this list
								To get the overall cost for traversing the total area.
			np_all_bbox (np.ndarray) : Contains the local bbox

		'''
		##decomp global map into smaller maps
		all_bbox, np_all_bbox = self.global_decomp()
		
		rotation = np.arange(0, 360, self.rotation_deg)
		##Best overall cost and rotated waypoints for each box are saved
		global_waypoints = {}
		global_costs = []
		global_times = []

		##for each box:
		print ("TOTAL NUM ITERATIONS: ", len(all_bbox))
		# for ii in range(len(all_bbox)):
		# for ii in range(len(all_bbox)-32, len(all_bbox)-16):
		for ii in range(60, 65):
			print ("PROCESSING: ", ii)
			box = all_bbox[ii]

			best_cost = 99999999999999999999

			##all the waypoints and rotation + costs are saved for each box
			save_waypoints = {}
			save_rotation_costs = np.empty((2, len(rotation)))
			for r in range(len(rotation)):
				rotated_bbox = sa.rotate(box, rotation[r]) ##Default, takes in rotation by degrees

				##Breaks down the global survey area into even smaller more managable regions
				x_bbox0, y_bbox0 = self.calc_bbox_lines(rotated_bbox, 
														self.search_radius,
														self.search_width)

				##Calculates mowing the lawn pattern
				waypts = self.calc_mowing_lawn(rotated_bbox, y_bbox0, start_end="left")
				centered_waypoints = sa.rotate(sh.Polygon(waypts), -rotation[r])

				##Visualize each rotated box
				# npbox = np_all_bbox[ii]
				# rotated0_np = np.array(rotated_bbox.exterior.coords)
				# plt.plot(npbox[:,0], npbox[:,1])
				# plt.plot(rotated0_np[:,0], rotated0_np[:,1])
				# plt.plot(centered_waypoints.exterior.coords.xy[0], 
				# 		 centered_waypoints.exterior.coords.xy[1])
				# plt.xlim(box.bounds[0] - 100, box.bounds[2] + 100)
				# plt.ylim(box.bounds[1] - 100, box.bounds[3] + 100)
				# plt.show()

				np_centered_waypoints = np.array(zip(centered_waypoints.exterior.coords.xy[0], 
													 centered_waypoints.exterior.coords.xy[1]))

				##Calculate the cost of traveling that path:
				cost = self.est_propulsion(np_centered_waypoints, best_cost, 
															est_uuv_pos=False, use_bound=True, 
															visualize=False)

				if visualize:
					save_rotation_costs[:, r] = [rotation[r], cost]
					save_waypoints[rotation[r]] = np_centered_waypoints

				if cost < best_cost:
					best_cost = cost
					best_waypoints = np_centered_waypoints
					best_rotation = rotation[r]
					best_time = cost

			##save the best cost and waypoints
			# print ("best cost: ", best_cost)
			# print ("best rotation: ", best_rotation)
			global_waypoints[ii] = best_waypoints[:-1, :]
			global_costs.append(best_cost)
			global_times.append(best_time)
			self.global_timesteps += best_time

			##Visualize all the local bbox results
			if visualize:
				self.visualize_local_results(save_rotation_costs, box, save_waypoints, ii, (self.global_timesteps)/3600)

		# global_times.append(self.global_timesteps)
		self.visualize_global_results(global_waypoints, global_costs, np_all_bbox, global_times)

		return global_waypoints, global_costs, np_all_bbox


	def visualize_local_results(self, save_rotation_costs, box, save_waypoints, boxnum, time_hrs):
		'''
		TODO: Comments

		Inputs:
			save_rotation_costs () : 
			box () : 
			save_waypoints () : 
		'''

		##Sort the array
		save_rotation_costs = save_rotation_costs[:, save_rotation_costs.argsort()[1]]
		print ("top 5 rotations: ", save_rotation_costs[0, 0:5] )
		print ("top 5 costs:", save_rotation_costs[1, 0:5] )

		small_bbox = np.array(zip(box.exterior.coords.xy[0], box.exterior.coords.xy[1]))
		small_bbox = small_bbox[:-1, :]

		##Visualize the currents
		xbound = [min(small_bbox[:,0]), max(small_bbox[:,0])]
		ybound = [min(small_bbox[:,1]), max(small_bbox[:,1])]


		d = grid_data(dfunc, xbound, ybound, 5, [], [])
		d = abs(d) - 2
		u = grid_data(ufunc, xbound, ybound, 5, d, [time_hrs])
		v = grid_data(vfunc, xbound, ybound, 5, d, [time_hrs])
		uv = grid_data(uvfunc, xbound, ybound, 5, d, [time_hrs])

		vmin = np.min(uv)
		vmax = np.max(uv)

		X,Y = np.meshgrid(np.linspace(xbound[0], xbound[1], u.shape[0], endpoint=True),
						  np.linspace(ybound[0], ybound[1], u.shape[1], endpoint=True))

		fig = plt.figure(1, figsize=(30, 20), dpi=160)
		fig.suptitle("box number (best solution) {0}".format(boxnum))
		for fignum in range(1, 7):
			ax = fig.add_subplot(2, 3, fignum)
			ax.scatter(X,Y, color='b', s=15)
			im1 = ax.quiver(X,Y, u, v, uv)
			fig.colorbar(im1)
			im1.set_clim(vmin, vmax)

			ax.title.set_text("Best Solution {0}  Rotation: {1}  Cost: {2}".format(fignum,
																				   save_rotation_costs[0,fignum-1],
																			 	   save_rotation_costs[1,fignum-1]))
			ax.set_xlim(xbound[0]-10, xbound[1]+10)
			ax.set_ylim(ybound[0]-10, ybound[1]+10)

			ordered_wp = save_waypoints[save_rotation_costs[0, fignum-1]][:-1, :]
			start_pt = plt.Circle((ordered_wp[0,0], ordered_wp[0,1]), 2, color='green')
			ax.add_artist(start_pt)
			end_pt = plt.Circle((ordered_wp[-1,0], ordered_wp[-1,1]), 2, color='red')
			ax.add_artist(end_pt)
			plt.plot(ordered_wp[:,0], ordered_wp[:,1])
		# plt.show()
		fig.savefig("./nominal_pics/box_{0}_best.png".format(boxnum))
		fig.clf()

		fig = plt.figure(1, figsize=(30, 20), dpi=160)
		fig.suptitle("box number (worst solution) {0}".format(boxnum))
		for fignum in range(1,7):
			ax = fig.add_subplot(2, 3, fignum)
			ax.scatter(X,Y, color='b', s=15)
			im1 = ax.quiver(X,Y, u, v, uv)
			fig.colorbar(im1)
			im1.set_clim(vmin, vmax)

			ax.title.set_text("Worst Solution {0}  Rotation: {1}  Cost: {2}".format(len(save_rotation_costs[0]) - fignum,
																					save_rotation_costs[0, -fignum],
																			 	    save_rotation_costs[1, -fignum]))
			ax.set_xlim(xbound[0]-10, xbound[1]+10)
			ax.set_ylim(ybound[0]-10, ybound[1]+10)

			ordered_wp = save_waypoints[save_rotation_costs[0, -fignum]][:-1, :]
			start_pt = plt.Circle((ordered_wp[0,0], ordered_wp[0,1]), 2, color='green')
			ax.add_artist(start_pt)
			end_pt = plt.Circle((ordered_wp[-1,0], ordered_wp[-1,1]), 2, color='red')
			ax.add_artist(end_pt)
			plt.plot(ordered_wp[:,0], ordered_wp[:,1])
		# plt.show()
		fig.savefig("./nominal_pics/box_{0}_worst.png".format(boxnum))
		fig.clf()


	def visualize_global_results(self, global_waypoints, global_cost, np_all_bbox, global_times):
		'''
		Visualizes the global solution for all of boxes

		Inputs:
			global_waypoints (Dict: key=int, item=np.ndarray) : 
			global_cost (List) : 
			np_all_bbox (np.ndarray) : 
			global_times (List) : contains the starting time for all the individual boxes
								to visualize the currents in the individual box
		'''
		cost_hours = sum(global_cost)/(3600)

		##need to create currents background
		xbound = [min(xy_dim[:,0]), max(xy_dim[:,0])]
		ybound = [min(xy_dim[:,1]), max(xy_dim[:,1])]

		## For each of the values in global_waypoints
		all_d_vis = np.zeros(())
		all_u_vis = np.zeros(())
		all_v_vis = np.zeros(())
		all_uv_vis = np.zeros(())
		for boxnum in np.sort(global_waypoints.keys()):
			##Grab the appropriate 

			d = grid_data(dfunc, xbound, ybound, 30, [], [])
			d = abs(d) - 2
			u = grid_data(ufunc, xbound, ybound, 30, d, global_times)
			v = grid_data(vfunc, xbound, ybound, 30, d, global_times)
			uv = grid_data(uvfunc, xbound, ybound, 30, d, global_times)

			##Add it to the right array
			

		vmin = np.min(uv)
		vmax = np.max(uv)

		X,Y = np.meshgrid(np.linspace(xbound[0], xbound[1], u.shape[0], endpoint=True),
						  np.linspace(ybound[0], ybound[1], u.shape[1], endpoint=True))

		fig = plt.figure(1, figsize=(30, 20), dpi=160)
		fig.suptitle("Global solution. Cost to travel all: {0} hours".format(cost_hours))
		ax = fig.add_subplot(1, 1, 1)

		np_all_bbox = np_all_bbox.reshape(-1, 2)
		plt.plot(np_all_bbox[:,0], np_all_bbox[:,1], color="black", linewidth=3)
		im1 = ax.quiver(X,Y, u, v, uv)
		fig.colorbar(im1)
		im1.set_clim(vmin, vmax)

		##for each box, plot the waypoints
		##TODO: Do the circles show up?
		for keys, items in global_waypoints.items():
			plt.plot(items[:,0], items[:,1])
			start_pt = plt.Circle((items[0,0], items[0,1]), 5, color='green')
			ax.add_artist(start_pt)
			end_pt = plt.Circle((items[-1,0], items[-1,1]), 5, color='red')
			ax.add_artist(end_pt)

		plt.xlim(xbound[0], xbound[1])
		plt.ylim(ybound[0], ybound[1])
		plt.show()

		fig.savefig("./nominal_pics/global_solution.png", bbox_inches='tight')


if __name__ == '__main__':
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



	width = haversine(place_bbox[0,0], place_bbox[0,1], place_bbox[1,0], place_bbox[1,1]) * 1000
	height = haversine(place_bbox[1,0], place_bbox[1,1], place_bbox[2,0], place_bbox[2,1]) * 1000

	xwidth = [-width/2, width/2]
	yheight = [-height/2, height/2]

	# u_area, v_area, uv_area, ufunc, vfunc, uvfunc = get_current_block(place_bbox, current_path)
	ufunc, vfunc, uvfunc = get_multi_current_block(place_bbox, currents_path1, currents_path2)
	d_area, dfunc = get_depth_block(place_bbox, topo_path)

	xy_dim = np.array([[-width/2, height/2], [width/2, height/2], [width/2, -height/2], [-width/2, -height/2]])

	M = Mowing_Lawn_BBox_Search(xy_dim)
	M.search()





	# global_bbox = sh.Polygon(place_bbox)
	# global_bbox = sh.Polygon([[0,0], [155,0], [155,100], [0,155]])

	# TM = Topological_Map()

	# map_contours = TM.get_map_contours()
	# for i in range(1, len(map_contours)):
	# print ("number of areas: ", len(map_contours))
	# area1 = map_contours[2].get_paths()[0].vertices
	# area2 = map_contours[3].get_paths()[1].vertices

	# plt.show()
	# plt.clf()

	# plt.plot(area1[:,0], area1[:,1], 'k')
	# plt.plot(area2[:,0], area2[:,1])
	# plt.show()

	# pdb.set_trace()

	##Connect them into a polygon now:
	# area12 = np.vstack((area1, np.flipud(area2)))
	# test_area = sh.Polygon(area12)
	# npta = np.array(test_area.exterior.coords)

	# plt.plot(npta[:,0], npta[:,1])
	# plt.show()

	# pdb.set_trace()

