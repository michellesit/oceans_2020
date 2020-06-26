import pickle

import rospkg
import matplotlib.pyplot as plt
from scipy.io import netcdf
import numpy as np
from numpy.random import randint, multivariate_normal
import shapely.geometry as sg

import pdb

'''
Utility trash ONLY functions to support trash_finder/scripts files
No UUV functions here

'''


def trash_2d_gauss(mu, sigma, num_soda_cans, dfunc):
	'''
	Generates a 2D normal distribution of trash from a single point

	Inputs:
		mu (list) : x,y mean of the normal distribution. Set to the center of the point
		sigma (np.ndarray) : covariance matrix (dim = 2x2)
		num_soda_cans (int) : number of soda can positions to generate
		z_range (list) : Lower and upper bound for depth
		dfunc (function) : interpolation function to get depths from generated positions
	Returns:
		sc_pos (np.ndarray) : normal distribution of trash positions in [z,x,y] format
	'''

	sc_x, sc_y = multivariate_normal(mu, sigma, num_soda_cans).T
	xy_pos = np.array(zip(sc_x, sc_y))
	z_pos = abs(dfunc(xy_pos))
	sc_pos = np.hstack((z_pos.reshape(-1,1), xy_pos))

	return sc_pos


def init_hotspots(trash_x_centers, trash_y_centers, input_sigma, num_soda_cans, dfunc):
	'''
	Create arbitrary 2D gaussians hotspots in a given area
	Returns a dictionary with:
		keys = soda can ID number (int)
		items = np.ndarray of soda can's positions throughout the timesteps

	Inputs:
		trash_x_centers (np.ndarray) : x coordinates for the centers of the trash hotspots
		trash_y_centers (np.ndarray) : y coordinates for the centers of the trash hotspots
		input_sigma (array) : (Optional) If provided, used to determine the standard deviation of the spread for all of the trash hotspots
		num_soda_cans (int) : number of hotspots
		dfunc (function) : depth function created from finder_utils

	Returns:
		trash_dict (Dict) : keys = soda can ID number, items = np.ndarray of soda can's
								positions throughout the timesteps

	
	'''
	trash_dict = {}
	for hotspot_idx in range(len(trash_x_centers)):
		if not input_sigma[0]:
			sigma = [[np.random.rand(), 0], [0, np.random.rand()]]
		else:
			sigma = input_sigma

		hotspot_trash_coords = trash_2d_gauss([trash_x_centers[hotspot_idx][0], trash_y_centers[hotspot_idx][0]], sigma, num_soda_cans, dfunc)
		trash_dict[hotspot_idx] = [hotspot_trash_coords]

	return trash_dict


def update_trash_pos(trash_dict, time_now_hrs, e, tracking=False):
	'''
	Takes dict of trash positions and updates them according to the currents + noise and random walk

	Inputs:
		trash_dict (Dict) : keys = soda can ID number, items = np.ndarray of soda can's
								positions throughout the timesteps
		time_now_hrs (float): time to update at in hrs
		e (Object) : Object (Env.py) that contains all the environment properties (currents, height, width)
		tracking (bool) : Set to true to keep track of all the soda can positions
							False = replace the trash position at each step
		
		ufunc, vfunc, dfunc (function) : interpolated currents and depth function from finder_utils
		width (int): width of the map in xy-dim
		height (int) : height of the map in xy-dim


	Returns:
		trash_dict (Dict) : updated with new soda can positions

	'''

	for group_ID in trash_dict.keys():

		cans = np.copy(trash_dict[group_ID][-1])
		cans = np.hstack((np.ones((cans.shape[0], 1))*time_now_hrs, cans))

		##80% of the time, the soda can follows the current + some noise
		##20% of the time, the soda can takes a step in a random direction
		random_walk_chance = np.random.rand()
		if random_walk_chance < 0.8:
			local_u = e.ufunc(cans) * np.random.normal(1, 0.5)
			local_v = e.vfunc(cans) * np.random.normal(1, 0.5)
		else:
			random_range = np.random.uniform(0, 2)
			local_u = np.random.uniform(-random_range, random_range, size=(cans.shape[0], 1))
			local_v = np.random.uniform(-random_range, random_range, size=(cans.shape[0], 1))

		##Move the soda cans by that amount (not technically correct, but an approximation)
		cans[:, 2] += local_u.flatten()
		cans[:, 3] += local_v.flatten()

		##TODO: TEST THIS
		##If the position of the soda can is at the bounds, 
		##then set the soda can position to boundary value
		cans[:,2][cans[:,2] > e.width/2] = e.width/2
		cans[:,2][cans[:,2] < -e.width/2] = -e.width/2
		cans[:,3][cans[:,3] > e.height/2] = e.height/2
		cans[:,3][cans[:,3] < -e.height/2] = -e.height/2

		new_z = -e.dfunc(cans[:, 2:4])
		new_cans = np.hstack((new_z.reshape((-1,1)), cans[:, 2:4]))

		##Add it to the dictionary and update last_pos
		if tracking:
			trash_dict[group_ID].append(new_cans)
		else:
			trash_dict[group_ID] = [new_cans]

	return trash_dict


def visualize_trash_flow(trash_dict, vis_clusters=True, map_dim=[]):
	'''
	Visualizes where each piece of trash has been over the whole simulation so far

	Inputs:
		trash_dict (Dict) : keys = soda can ID number, items = np.ndarray of soda can's
								positions throughout the timesteps
		vis_clusters (bool) : True = draws the convex hull + a buffer around each trash hotspot 
		map_dim (array): [[xmin, xmax], [ymin, ymax]]

	'''

	if map_dim:
		plt.xlim(map_dim[0])
		plt.ylim(map_dim[1])

	for trash_group_ID in trash_dict.keys():
		group = np.array(trash_dict[trash_group_ID])
		group_color = tuple(np.random.rand(1,3)[0])

		for trash_piece in range(group.shape[1]):
			movement = np.array(trash_dict[trash_group_ID])
			plt.plot(movement[:, trash_piece, 1], movement[:, trash_piece, 2], color=group_color, marker='_', linewidth=2)

		if vis_clusters:
			convexhull_a = sg.MultiPoint(group[-1][:, 1:3]).convex_hull
			buffer_a = convexhull_a.buffer(5)
			ax, ay = buffer_a.exterior.coords.xy
			plt.plot(ax, ay, color=group_color)


def visualize_trash_step(trash_dict, vis_clusters=True, map_dim=[]):
	'''
	Visualizes the last soda can position from all of the groups

	Inputs:
		trash_dict (Dict) : keys = soda can ID number, items = np.ndarray of soda can's
								positions throughout the timesteps
		vis_clusters (bool) : True = draws the convex hull + a buffer around each trash hotspot 
		map_dim (array): [[xmin, xmax], [ymin, ymax]]
	'''

	if map_dim:
		plt.xlim(map_dim[0])
		plt.ylim(map_dim[1])

	for trash_group_ID in trash_dict.keys():
		group = np.array(trash_dict[trash_group_ID])
		group_color = tuple(np.random.rand(1,3)[0])

		for trash_piece in range(group.shape[1]):
			movement = np.array(trash_dict[trash_group_ID])
			plt.plot(movement[-1, trash_piece, 1], movement[-1, trash_piece, 2], color=group_color, marker='o')

		if vis_clusters:
			convexhull_a = sg.MultiPoint(group[-1][:, 1:3]).convex_hull
			buffer_a = convexhull_a.buffer(5)
			ax, ay = buffer_a.exterior.coords.xy
			plt.plot(ax, ay, color=group_color)


def main():
	rospack = rospkg.RosPack()
	trash_finder_path = rospack.get_path("trash_finder")
	data_path = rospack.get_path("data_files")

	config = pickle.load(open(trash_finder_path + '/config/demo_configs.p', 'rb'))

	##hydrodynamic current
	currents1 = config['current_file1']
	currents_path1 = data_path + '/' + currents1
	currents2 = config['current_file2']
	currents_path2 = data_path + '/' + currents2
	current_path = data_path+'/ca_subCA_das_2020010615.nc'

	##bathymetry model
	topo = config['topo_file']
	topo_path = data_path + '/' + topo

	all_locations = pickle.load( open(trash_finder_path + "/config/locations.p", "rb"))
	place_bbox = all_locations["mission_bay_flatter_bbox"]

	width = haversine(place_bbox[0,0], place_bbox[0,1], place_bbox[1,0], place_bbox[1,1]) * 1000
	height = haversine(place_bbox[1,0], place_bbox[1,1], place_bbox[2,0], place_bbox[2,1]) * 1000
	xy_dim = np.array([[-width/2, height/2], [width/2, height/2], [width/2, -height/2], [-width/2, -height/2]])

	d_area, dfunc = get_depth_block(place_bbox, topo_path)
	ufunc, vfunc, uvfunc = get_multi_current_block(place_bbox, currents_path1, currents_path2)

