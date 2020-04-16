import numpy as np
from trash_utils.finder_utils import get_depth_block

import pdb

def man_create_soda_pos(num_cans, bbox, topo_path, 
						xbounds, ybounds, zbounds,
						dist_type="depth_file"):
	'''
	Used for testing the validity of the environment
	Generates soda can x,y,z positions to be read into randomize_init_soda_cans.py

	Input:
		num_cans (int): number of cans to generate
		bbox (np.ndarray): 4x2 array with lat/lon float coordinates for the area of interest
			Coordinates should be top left, top right, bottom right, bottom left
		topo_path (string): path to netcdf file containing the hydrodynamic data

		xbounds (array of ints): x coordinate range for distribution
		ybounds (array of ints): y coordinate range for distribution
		zbounds (array of ints): x coordinate range for distribution

		dist_type (str): specifies how the depth values are created
			depth_file : depth is the bottom of the seafloor according to the topo_file
			grid : depth is generated randomly using z_bounds values

	Output:
		cans_pos (np.ndarray): num_cans x 3 array for x,y,z(depth) pos

	'''
	x = np.arange(xbounds[0], xbounds[1], num_cans)
	y = np.arange(ybounds[0], ybounds[1], num_cans)

	if dist_type == "depth_file":
		##Get the z dimension from the currents file:
		##Write method to get the z dimensions from the currents file
		depth_area, depth_func = get_depth_block(bbox, topo_path)

		cans_x_pos, cans_y_pos = np.meshgrid(x, y)
		cans_x_pos = cans_x_pos.flatten()
		cans_y_pos = cans_y_pos.flatten()

		grid = zip(cans_x_pos, cans_y_pos)
		cans_z_pos = depth_func(grid)+3.0

	else:
		z = -np.arange(zbounds[0], xbounds[1], num_cans)
		cans_x_pos, cans_y_pos = np.meshgrid(cans_x_pos, cans_y_pos, cans_z_pos)
		cans_x_pos = cans_x_pos.flatten()
		cans_y_pos = cans_y_pos.flatten()
		cans_z_pos = cans_z_pos.flatten()

	cans_pos = np.vstack((cans_x_pos, cans_y_pos, cans_z_pos))

	return cans_pos