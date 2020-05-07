import pickle
from math import atan2, sin, cos

import rospkg
from scipy.io import netcdf
import numpy as np
import shapely.geometry as sh
import shapely.affinity as sa
import matplotlib.pyplot as plt

from trash_utils.haversine_dist import haversine
from trash_utils.finder_utils import (get_current_block,
									  get_depth_block,
									  grid_data,
									  find_bound_idx,
									  xy_to_lat_lon)

import pdb

'''
Goal: discretizes the map into a lawn mower pattern in the direction of interest
	Takes the shortest dimension (which means the rows will be long. ie less turns)
	Calculates the waypoints needed to complete the pattern

TODO: incorporate contours/gradient difference into global decomposition
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
		self.goal_threshold = 1			##meters
		self.uuv_heading = 0			##radians
		self.uuv_speed = 2.5722 		##meters/second
		self.uuv_position = bbox[0,:]

		self.global_bbox = sh.Polygon(bbox)


	def global_decomp(self):
		'''
		Breaks down the global bbox into smaller more managable boxes

		'''
		x_bbox, y_bbox = self.calc_bbox_lines(self.global_bbox, 
											  self.global_bbox_height_max,
											  self.global_bbox_width_max)

		print ("x_bbox: ", len(x_bbox))
		print ("y_bbox: ", len(y_bbox))

		##TODO: This is time consuming for many points. Get rid of the two for loops
		##CHECK: Rearrange the points to be top left, top right, bottom right, bottom left
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

		np_all_bbox = np.array(np_all_bbox).reshape((-1,2))
		
		##Visualize all the bbox here:
		# print (all_bbox)
		print (np_all_bbox)

		plt.plot(place_bbox[:,0], place_bbox[:,1])
		plt.plot(np_all_bbox[:,0], np_all_bbox[:,1])
		# plt.show()

		return all_bbox


	def calc_bbox_lines(self, bbox, height_radius, width_radius):
		'''
		

		Input:
		bbox = Polygon([upper left, upper right, bottom right, bottom left coords])
		self.search_radius = height of the search rows

		Output:
		x_bbox, y_bbox = 

		Make sure the boxes are odd rows with odd number of boxes?

		'''
		minx, miny, maxx, maxy = bbox.bounds
		bbox_height = abs(maxy - miny)
		bbox_width = abs(maxx - minx)

		num_height_lines = bbox_height//height_radius
		num_width_lines = bbox_width//width_radius

		print (num_height_lines)
		print (num_width_lines)

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
		Calculate the intersection of the lines with the bbox:
		For each pair of x_bbox lines, calculate intersection with bbox
		'''

		all_lines = []
		all_intersections = []
		for i in range(y_bbox.shape[0]):
			lines = sh.LineString([ [bbox.bounds[0]-(bbox.bounds[0]*0.4), y_bbox[i]],
									[bbox.bounds[2]+(bbox.bounds[2]*0.4), y_bbox[i]] ])
			intersections = bbox.intersection(lines)

			all_lines.append(lines)
			all_intersections.append(np.array(intersections))

		##TODO: Should add in a check here to make sure there are intersecting lines
		all_lines = sh.MultiLineString(all_lines)
		# print ("all lines:")
		# print(all_lines)

		# print ("all intersections: ")
		# print (all_intersections)

		##TODO: add in logic to make sure the coordinates make sense?
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
			
		# print ("waypoints: ", np.array(waypoints))

		return np.array(waypoints)


	def est_propulsion(self, waypoints, est_uuv_pos=True):
		'''
		Do some vector math to calculate what direction/where you'd end up based 
			on currents and the uuv's vector heading/power
	
		uuv_vector (list(float, float)): magnitude and direction (rad)
		current_vector list(float, float): magnitude and direction (rad)
		'''
		
		##Given some waypoint we are heading to
		##for each waypoint in the path, iteratively calculate how long it would take to get there

		##for each 1 second (since we are traveling at 5knots = 2.5722m/sec)
		##uuv_heading = some angle
		##uuv_speed = constant for now

		##desired_heading = some angle compared to this

		##currents_speed = uv at the current position
		##currents_heading = np.tan(u,v)

		##vector math
		##resulting speed = 
		##resulting heading =

		##Set uuv position to be the first waypoint
		##Otherwise, use the uuv's current position as its starting location
		if est_uuv_pos:
			self.uuv_position = waypoints[1]

		timesteps = 0
		for wp in range(len(waypoints)):
			goal = waypoints[wp]
			print ("NEW WAYPOINT: ", goal)

			# goal = waypoints[2]
			while abs(np.linalg.norm(self.uuv_position - goal)) > self.goal_threshold:
				##Calculate the heading to that location

				print ("Waypoint: ", wp)

				desired_heading = atan2(goal[1] - self.uuv_position[1] - goal[1], goal[0] - self.uuv_position[0])
				goal_u = cos(desired_heading) * np.linalg.norm(goal - self.uuv_position)
				goal_v = sin(desired_heading) * np.linalg.norm(goal - self.uuv_position)

				z = abs(dfunc(self.uuv_position))
				current_u = ufunc([z, self.uuv_position[0], self.uuv_position[1]])[0][0]
				current_v = vfunc([z, self.uuv_position[0], self.uuv_position[1]])[0][0]
				current_heading = atan2(current_v, current_u)

				##vector math
				desired_u = goal_u + current_u
				desired_v = goal_v - current_v
				resulting_heading = atan2(desired_v, desired_u)

				uuv_u = cos(resulting_heading) * self.uuv_speed
				uuv_v = sin(resulting_heading) * self.uuv_speed

				resulting_speed = np.array([uuv_u + current_u, uuv_v + current_v])
				

				print ("desired_heading: ", np.rad2deg(desired_heading))
				print ("goal: ", goal)
				print ("uuv_position: ", self.uuv_position)
				print ("goal_u: ", goal_u)
				print ("goal_v: ", goal_v)

				print ("z: ", z)
				print ("current_u: ", current_u)
				print ("current_v: ", current_v)
				print ("currents_heading: ", np.rad2deg(current_heading))

				print ("desired_u: ", desired_u)
				print ("desired_v: ", desired_v)
				print ("resulting_heading: ", np.rad2deg(resulting_heading))

				print ("uuv_u: ", uuv_u)
				print ("uuv_v: ", uuv_v)

				print ("resulting_speed: ", resulting_speed)
				print ("new end pos: ", self.uuv_position + resulting_speed)

				# print ("update uuv_position with this new info")
				plt.clf()


				fig, ax = plt.subplots()
				plt.plot(waypoints[:,0], waypoints[:,1])
				ax.quiver(self.uuv_position[0], self.uuv_position[1], uuv_u, uuv_v, scale=1, scale_units='xy', color='green') ##uuv

				wp_c = plt.Circle((goal[0], goal[1]), 1, color='black') ##waypoint
				ax.add_artist(wp_c)
				uuv_end = plt.Circle((self.uuv_position[0] + resulting_speed[0], self.uuv_position[1] + resulting_speed[1]), 1, color="blue")
				ax.add_artist(uuv_end)

				plt.show()

				self.uuv_position += resulting_speed
				self.uuv_heading = resulting_heading

				##add to timestep count (time)
				timesteps += 1
				print ("dist to goal: ", abs(np.linalg.norm(self.uuv_position - goal)))
				print ('continue while loop: ', abs(np.linalg.norm(self.uuv_position - goal)) > self.goal_threshold)

				pdb.set_trace()
	
		print ("timesteps: ", timesteps)


	def search(self):
		'''
		Calculate waypoints for each of the smaller bbox areas
		for each bbox in all_bbox:
			Rotate by some amount
			Calculate the waypoints for each
			Calculate the cost of traveling that path
			Keep the best cost and rotation

		'''
		##decomp global map into smaller maps
		all_bbox = self.global_decomp()
		
		rotation = np.arange(0, 360, self.rotation_deg)
		best_cost = 99999999999999999999

		##for each box:
		# for ii in range(len(all_bbox)):
		for ii in range(1):
			print ("ii: ", ii)
			box = all_bbox[ii]

			for r in range(len(rotation)):
				# rotated_bbox = sa.rotate(box, rotation[r]) ##By default, takes in rotation by degrees
				rotated_bbox = sa.rotate(box, 10) ##By default, takes in rotation by degrees
				# print ("box: ")
				# print(box)

				# print ("rotated bbox: ")
				# print (rotated_bbox)

				rotated0_np = np.array(rotated_bbox.exterior.coords)

				##Breaks down the survey area into even smaller more managable regions
				x_bbox0, y_bbox0 = self.calc_bbox_lines(rotated_bbox, 
														self.search_radius,
														self.search_width)

				##Calculates mowing the lawn pattern
				waypts = self.calc_mowing_lawn(rotated_bbox, y_bbox0, start_end="left")
				# centered_waypoints = sa.rotate(sh.Polygon(waypts), -rotation[r])
				centered_waypoints = sa.rotate(sh.Polygon(waypts), -10)

				# plt.plot(place_bbox[:,0], place_bbox[:,1])
				# plt.plot(rotated0_np[:,0], rotated0_np[:,1])
				# plt.plot(centered_waypoints.exterior.coords.xy[0], 
				# 		 centered_waypoints.exterior.coords.xy[1])
				# plt.show()

				np_centered_waypoints = np.array(zip(centered_waypoints.exterior.coords.xy[0], 
													 centered_waypoints.exterior.coords.xy[1]))

				##Calculate the cost of traveling that path:
				cost = self.est_propulsion(np_centered_waypoints, True)
				print ("WE ARE OUTSIDE OF EST_PROPULSION")

				if cost < best_cost:
					best_cost = cost
					best_waypoints = np_centered_waypoints

				print ("best cost: ", best_cost)

		return best_waypoints


if __name__ == '__main__':
	rospack = rospkg.RosPack()
	trash_finder_path = rospack.get_path("trash_finder")
	data_path = rospack.get_path("data_files")

	config = pickle.load(open(trash_finder_path + '/config/demo_configs.p', 'rb'))

	##hydrodynamic current
	current_path = data_path+'/ca_subCA_das_2020010615.nc'
	current_data = netcdf.NetCDFFile(current_path)
	depth = current_data.variables['depth'][:].copy()
	current_data.close()

	##bathymetry model
	topo = config['topo_file']
	topo_path = data_path + '/' + topo

	all_locations = pickle.load( open(trash_finder_path + "/config/locations.p", "rb"))
	place_bbox = all_locations["mission_bay_flatter_bbox"]



	# min_lat = min(place_bbox[:, 0])
	# max_lat = max(place_bbox[:, 0])
	# min_lon = min(place_bbox[:, 1])
	# max_lon = max(place_bbox[:, 1]) 

	width = haversine(place_bbox[0,0], place_bbox[0,1], place_bbox[1,0], place_bbox[1,1]) * 1000
	height = haversine(place_bbox[1,0], place_bbox[1,1], place_bbox[2,0], place_bbox[2,1]) * 1000

	xwidth = [-width/2, width/2]
	yheight = [-height/2, height/2]

	u_area, v_area, uv_area, ufunc, vfunc, uvfunc = get_current_block(place_bbox, current_path)
	d_area, dfunc = get_depth_block(place_bbox, topo_path)

	depths = grid_data(dfunc, xwidth, yheight, 10, [])
	depths += 1
	depths_flat = depths.flatten()

	# ##these are interpolated grids of 
	u = grid_data(ufunc, xwidth, yheight, 10, depths)
	v = grid_data(vfunc, xwidth, yheight, 10, depths)
	uv = grid_data(uvfunc, xwidth, yheight, 10, depths)

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

