from ConfigParser import SafeConfigParser

import numpy as np
from scipy.interpolate import RegularGridInterpolator, interpn
from scipy.io import netcdf

import pdb
import rospkg

from trash_utils.haversine_dist import haversine

import matplotlib.pyplot as plt

'''
Utility functions to support the trash_finder/scripts files

'''

def xyz_to_lat_lon(bbox, x,y):
	'''
	Converts x,y coordinates to lat/lon coordinates

	Input:
		bbox (np.ndarray): 4x2 array with lon/lat float coordinates for the area of interest
			Coordinates should be top left, top right, bottom right, bottom left
			Each bbox corner should be in [lon, lat] order
		x (float): value from coordinate system to convert to lat
		y (float): value from coordinate system to convert to lon

	Returns:
		lat (float): corresponding lat value to x coordinate
		lon (float): corresponding lon value to y coordinate


	##TODO: CHECK THIS IS CORRECT
	'''
	width = haversine(bbox[0,0], bbox[0,1], bbox[1,0], bbox[1,1])
	height = haversine(bbox[1,0], bbox[1,1], bbox[2,0], bbox[2,1])

	##assert the x and y values are valid
	##depends on whether the values are zeroed or centered
	if not ((0 <= abs(x) <= width/2)):
		raise Exception ("X VALUE OUTSIDE BOUNDS")
	if not ((0 <= abs(y) <= height/2)):
		raise Exception ("Y VALUE OUTSIDE BOUNDS")

	min_lat = min(bbox[:, 0])
	max_lat = max(bbox[:, 0])
	min_lon = min(bbox[:, 1])
	max_lon = max(bbox[:, 1])

	lat = (x * width * max_lat) + min_lat
	lon = (y * height * max_lon) + min_lon

	pdb.set_trace()

	return lat, lon


def lat_lon_to_xy(bbox, lat, lon):
	'''
	Converts lat/lon coordinates to x,y coordinates

	Input:
		bbox (np.ndarray): 4x2 array with lon/lat float coordinates for the area of interest
			Coordinates should be top left, top right, bottom right, bottom left
			Each bbox corner should be in [lon, lat] order
		lat (float): lattitude value to convert to x coordinate
		lon (float): longitude value to convert to y coordinate
	Return:
		x (float): corresponding x value to the lat coordinate
		y (float): corresponding y value to the lon coordinate


	##TODO: CHECK THIS IS CORRECT
	'''
	##assert that the values are valid within limits
	if not ((0 <= abs(lat) <= 90)):
		raise Exception ("INVALID LAT INPUT")
	if not ((0 <= abs(lon) <= 180)):
		raise Exception ("INVALID LON INPUT")	

	width = haversine(bbox[0,0], bbox[0,1], bbox[1,0], bbox[1,1])
	height = haversine(bbox[1,0], bbox[1,1], bbox[2,0], bbox[2,1])

	min_lat = min(bbox[:, 0])
	max_lat = max(bbox[:, 0])
	min_lon = min(bbox[:, 1])
	max_lon = max(bbox[:, 1])

	x = (lat - min_lat) * (width/max_lon) * 1000
	y = (lon - min_lon) * (height/max_lon) * 1000

	pdb.set_trace()

	return x,y


def find_bound_idx(data, coord):
	'''
	Given a point and data array that may not have the specific point, finds the two values
		that bound the point. Returns the two index values of those points
	This method is written to handle negative data and values
	
	Input:
		data (np.ndarray): array containing all the data
		coord (list): list containing some data point that we need to find the bounds of
			Could be either a single value (depth) OR a min and max value (currents)

	Returns:
		(np.ndarray): np array containing:
			min (int): idx value of the minimum bound
			max (int): idx value of the maximum bound
	
	'''
	##make absolute value to handle negative numbers
	abs_data = np.sort(abs(data))
	abs_coord = np.absolute(coord)

	if len(coord) == 1: ##THIS ONE HAS TO BE TESTED
		start = max(np.where(abs_data<=abs_coord)[0])
		end = min(np.where(abs_data>=abs_coord)[0])

	else:
		min_val = min(abs_coord)
		max_val = max(abs_coord)

		start = max(np.where(abs_data<=min_val)[0])
		end = min(np.where(abs_data>=max_val)[0])

	return np.array((min(start, end), max(start, end)))


def grid_data(interp_f, x_bound, y_bound):
	'''
	Input:
		interp_f : Scipy RegularGridInterpolator function that interpolates the data
			Takes an array of coordinates as input
		x_bound (list) : list of 

		dim (array): size=(1,2), gives the x,y dimension of the output

	Returns:
		interp_data (np.array): shape=(dim), interpolated data of the provided coordinates

	'''
	##meshgrid the data based on dim size
	##TODO: Figure out how to incorporate x and y bound
	x = np.arange(x_bound[0], x_bound[1]+1)
	y = np.arange(y_bound[0], y_bound[1]+1)

	xv, yv = np.meshgrid(x,y)
	grid = zip(xv.flatten(), yv.flatten())

	##feed it into the interpolation function to get output data of the same size
	interp_data = interp_f(grid)

	return interp_data.reshape(abs(x_bound[1]-x_bound[0]), abs(y_bound[1] - y_bound[0]))


def get_depth_block(bbox, topo_file):
	'''
	Input:
		bbox (np.ndarray): 4x2 array with lat/lon float coordinates for the area of interest
			Coordinates should be top left, top right, bottom right, bottom left
		topo_file (string): path to netcdf file containing the hydrodynamic data

	Returns:
		depth_area (np.ndarray): (n,m) array of the depths within the bbox
		depth_func : interpolation function that outputs the depth within
					 the array [+-width/2, +-height/2]

	TODO: plot the output of this and check it looks correct	
	'''
	topo_depths = netcdf.NetCDFFile(topo_file)
	lat = topo_depths.variables['lat'][:].copy()
	lon = topo_depths.variables['lon'][:].copy()
	depths = topo_depths.variables['Band1'][:].copy()
	topo_depths.close()

	min_lat = min(bbox[:, 0])
	max_lat = max(bbox[:, 0])
	min_lon = min(bbox[:, 1])
	max_lon = max(bbox[:, 1])

	##Finding the lon bounds is different cause the values are negative
	lon_idx = find_bound_idx(lon, [min_lon, max_lon])
	lon_idx = np.sort(lon.shape[0]-1-lon_idx)
	lat_idx = find_bound_idx(lat, [min_lat, max_lat])

	##pull that data from the depth file:
	depths_area = depths[lat_idx[0]:lat_idx[-1]+1, lon_idx[0]:lon_idx[-1]+1]

	##interpolation things:
	width = haversine(bbox[0,0], bbox[0,1], bbox[1,0], bbox[1,1])*1000
	height = haversine(bbox[1,0], bbox[1,1], bbox[2,0], bbox[2,1])*1000

	x = np.linspace(-width/2, width/2, depths_area.shape[0], endpoint = True)
	y = np.linspace(-height/2, height/2, depths_area.shape[1], endpoint = True)

	depths_func = RegularGridInterpolator((x, y), depths_area)

	return depths_area, depths_func


def get_current_block(bbox, current_file):
	'''
	Input:
		bbox (np.ndarray): 4x2 array with lon/lat float coordinates for the area of interest
			Coordinates should be top left, top right, bottom right, bottom left
			Each bbox corner should be in [lon, lat] order
		current_file (string): path to netcdf file containing the hydrodynamic data

	Returns:
		u_area (np.ndarray): (n,m) array of the currents in the u direction within the bbox
		v_area (np.ndarray): (n,m) array of the currents in the v direction within the bbox
		u_func : interpolation function that outputs the u value at the specified array[depth, x, y] point
		v_func : interpolation function that outputs the v value at the specified array[depth, x, y] point

	'''
	all_currents = netcdf.NetCDFFile(current_file)
	lon = all_currents.variables['lon'][:].copy()
	lon -= 360
	lat = all_currents.variables['lat'][:].copy()
	u = all_currents.variables['u'][:].copy()
	v = all_currents.variables['v'][:].copy()
	depths = all_currents.variables['depth'][:].copy()
	all_currents.close()

	##find the bounds
	min_lon = min(bbox[:, 1])
	max_lon = max(bbox[:, 1])
	min_lat = min(bbox[:, 0])
	max_lat = max(bbox[:, 0])

	##Finding the lon bounds is different cause the values are negative
	lon_idx = find_bound_idx(lon, [min_lon, max_lon])
	lon_idx = np.sort(lon.shape[0]-1-lon_idx)
	lat_idx = find_bound_idx(lat, [min_lat, max_lat])

	X = lon[lon_idx[0]:lon_idx[-1]+1]
	Y = lat[lat_idx[0]:lat_idx[-1]+1]

	##pull those coordinates from the current file
	u_area = (u[0, :, lat_idx[0]:lat_idx[-1]+1, lon_idx[0]:lon_idx[-1]+1])
	v_area = (v[0, :, lat_idx[0]:lat_idx[-1]+1, lon_idx[0]:lon_idx[-1]+1])

	##Filter out curernts that are greater than -9999:
	u_area[u_area<-800] = 0.0
	v_area[v_area<-800] = 0.0

	uv_area = np.hypot(u_area, v_area)

	##This is all interpolation things
	##allocate arrays for width, height
	width = haversine(bbox[0,0], bbox[0,1], bbox[1,0], bbox[1,1])*1000
	height = haversine(bbox[1,0], bbox[1,1], bbox[2,0], bbox[2,1])*1000
	
	##create grid with lat-lon coordinates
	x = np.linspace(-width/2, width/2, u_area.shape[1], endpoint = True)
	y = np.linspace(-height/2, height/2, u_area.shape[2], endpoint = True)

	u_func = RegularGridInterpolator((depths, x, y), u_area)
	v_func = RegularGridInterpolator((depths, x, y), v_area)
	uv_func = RegularGridInterpolator((depths, x, y), uv_area)

	return u_area, v_area, uv_area, u_func, v_func, uv_func


rospack = rospkg.RosPack()
trash_finder_path = rospack.get_path("trash_finder")
data_path = rospack.get_path("data_files")

config = SafeConfigParser()
config.read(trash_finder_path + '/scripts/demo_configs.ini')

currents1 = config.get('CURRENTS', 'current_file')
currents_path = data_path + '/' + currents1

topo = config.get('TOPO', 'topo_file')
topo_path = data_path + '/' + topo

places = np.array([
	[config.get('COORDS', 'loc1_lat'), config.get('COORDS', 'loc1_lon')],
	[config.get('COORDS', 'loc2_lat'), config.get('COORDS', 'loc2_lon')],
	[config.get('COORDS', 'loc3_lat'), config.get('COORDS', 'loc3_lon')],
	[config.get('COORDS', 'loc4_lat'), config.get('COORDS', 'loc4_lon')]
	], dtype=float)
print ("PLACES: ", places)
get_current_block(places, currents_path)
# get_depth_block(places, topo_path)
