import pickle

import rospkg
import matplotlib.pyplot as plt
from scipy.io import netcdf
import numpy as np
from numpy.random import randint, multivariate_normal

from trash_utils.haversine_dist import haversine
from trash_utils.finder_utils import (get_depth_block, 
									  get_multi_current_block,
									  lat_lon_to_xy)
from search import *

'''
2D sim for approximating where the trash will be based on local currents
Generates trash positions and visualizes them

'''

def trash_init(mu, sigma, num_soda_cans, dfunc):
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


def trash_sim(mean, cov, num_soda_cans, dfunc, ufunc, vfunc, width, height):
	'''
	Approximates where the trash will go based on currents
	Trash position is assumed to follow the currents, so at each time step,
		the new trash position is in the direction of the current

	Inputs:
		mean (list) : starting coordinates of the trash distribution
		cov (np.ndarray) : 2x2 covariance matrix for normal distribution
		num_soda_cans (int) : number of soda cans to generate in the sim
		dfunc (function) : interpolation function to get depths from generated positions
		ufunc (function) : interpolation function to get u currents from generated positions
		vfunc (function) : interpolation function to get v currents from generated positions
		width (float) : xy-frame width of the search area
		height (float) : xy-frame height of the search area
	Returns:
		all_trash_pos (Dict) : keys = soda can ID number, 
							   item = np.ndarray of positions

	'''
	##Generate a bunch of soda cans
	trash_pos = trash_init(mean, cov, num_soda_cans, dfunc)

	##Generate dictionary of soda can positions:
	all_trash_pos = {}
	all_last_pos = trash_pos
	for ii in range(trash_pos.shape[0]):
		all_trash_pos[ii] = [trash_pos[ii]]

	##tstep is in hours
	for tstep in np.arange(0, 145, 0.08):
		for sc in range(trash_pos.shape[0]):
			##Get the local current values
			point = np.hstack((tstep, all_last_pos[sc]))
			local_u = ufunc(point)
			local_v = vfunc(point)

			##Move the soda cans by that amount (not technically correct, but an approximation)
			new_x = all_last_pos[sc][1] + local_u
			new_y = all_last_pos[sc][2] + local_v

			##if the position of the soda can is at the bounds, 
			##then set the soda can position to boundary value
			if abs(new_x) > width/2:
				if new_x < 0:
					new_x = -width/2
				else:
					new_x = width/2

			if abs(new_y) > height/2:
				if new_y < 0:
					new_y = -height/2
				else:
					new_y = height/2

			new_z = -dfunc([new_x[0], new_y[0]])
			new_pos = [new_z[0], new_x[0], new_y[0]]

			##Add it to the dictionary and update last_pos
			all_trash_pos[sc].append(new_pos)
			all_last_pos[sc] = new_pos

	return all_trash_pos


def visualize_trash_flow(all_trash_pos, xy_dim):
	'''
	Visualizes the soda can paths

	Inputs:
		all_trash_pos (Dict) : keys = soda can ID number, items = np.ndarray of soda can's
								positions throughout the timesteps
		xy_dim (np.ndarray) : bounding box of the search area
	'''

	##visualize the global map
	gbox = np.vstack((xy_dim, xy_dim[0,:]))
	plt.plot(gbox[:,0], gbox[:,1])

	for trash in range(len(all_trash_pos.keys())):
		movement = np.array(all_trash_pos[trash])
		plt.plot(movement[:,1], movement[:,2], linewidth=2)

	##Prints number next to step
	for step in range(0, movement.shape[0], 10):
		plt.text(movement[step, 1], movement[step, 2], step)

	plt.show()


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


	# mean = [40, 50]
	distribution_point_latlon = [32.757, -117.260]
	[x_mean, y_mean] = lat_lon_to_xy(place_bbox, distribution_point_latlon[0], distribution_point_latlon[1])
	mean = [x_mean, y_mean]
	cov = np.array([[1.5, 0], [0, 1.5]])
	num_soda_cans = 10

	# M = Mowing_Lawn_BBox_Search(xy_dim)
	# all_bbox, np_all_bbox = M.global_decomp()
	# reshaped = np_all_bbox.reshape((-1,2))
	# plt.plot(reshaped[:,0], reshaped[:,1], color="black", linewidth=3)


	trash_movements = trash_sim(mean, cov, num_soda_cans, dfunc, ufunc, vfunc, width, height)
	visualize_trash_flow(trash_movements, xy_dim)

main()