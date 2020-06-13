import pickle

import rospkg
import matplotlib.pyplot as plt
from scipy.io import netcdf
import numpy as np
from numpy.random import randint, multivariate_normal


'''
Utility trash functions to support trash_finder/scripts files

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