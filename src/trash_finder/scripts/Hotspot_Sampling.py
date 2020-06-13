import pickle
from math import atan2, sin, cos

import rospkg
from scipy.io import netcdf
import numpy as np
import shapely.geometry as sg
import shapely.affinity as sa
from shapely.ops import nearest_points
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
		
		sigma = [[0.5, 0.8], [0.5, 0.8]]
		num_soda_cans = 10
		trash_dict = {}

		for hotspot_idx in range(self.num_hotspots):
			hotspot_trash_coords = trash_2d_gauss([trash_x_centers[hotspot_idx][0], trash_y_centers[hotspot_idx][0]], sigma, num_soda_cans, self.dfunc)
			trash_dict[hotspot_idx] = [hotspot_trash_coords]

		return trash_dict


	def est_propulsion_2d(self, waypoints, bound_cost, est_uuv_pos=True, use_bound=False, visualize=False):
		'''
		For each waypoint in the path, iteratively calculate how long it would take to get there.
		Do some vector math to calculate what direction/where you'd end up based 
			on currents and the uuv's vector heading/power
		Calculates in 3D

		Visualiztion shows an animation of the UUV's progress moving toward each waypoint

		Calculates for each 1 second (since we are traveling at 5knots = 2.5722m/sec)
		uuv_speed = constant for now

		Inputs:
			waypoints (np.ndarray) : Ordered 3D waypoints that the UUV needs to take.
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
					uuv_end = plt.Circle((self.uuv_position[0] + resulting_speed[0], self.uuv_position[1] + resulting_speed[1]), 1, color="blue") #Change this circle to something else
					ax.add_artist(uuv_end)
					plt.pause(0.05)

				self.uuv_position += resulting_speed
				self.uuv_heading = resulting_heading

				##add to timestep count (time)
				timesteps += 1
				if use_bound and timesteps > bound_cost:
					return 99999999999999999

		return timesteps


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

		## Create hotspots of trash
		hotspot_dict = self.init_hotspots(width, height)
		print (hotspot_dict)
		pdb.set_trace()

		## Create a matrix containing cost to get from hotspot to hotspot
		## Create a hotspot grid, which contains the cost of how long it would take to do cc in the hotspot
		total_time_duration = 15*4*24*4
		cost_matrix = np.empty((total_time_duration, self.num_hotspots, self.num_hotspots))
		cc_cost_matrix = np.empty((total_time_duration, self.num_hotspots, 1))
		paths_matrix = []

		## For every 15 minutes over 4 days:
		for ts in range(total_time_duration):
			convex_hotspots = []
			original_hotspots = []

			##For each hotspot
			for a_idx in range(self.num_hotspots-1):
				## Get convex hull of each of the hotspots plus some buffer
				convexhull_a = sg.MultiPoint(hotspot_dict[a_idx][-1][:, 1:3]).convex_hull
				buffer_a = convexhull_a.buffer(5)
				ax, ay = buffer_a.exterior.coords.xy

				##Calculate how much time it would take to do cc on that hotspot



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