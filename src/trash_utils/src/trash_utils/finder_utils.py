from ConfigParser import SafeConfigParser

import numpy as np
from scipy.interpolate import griddata
from scipy.io import netcdf

import pdb
import rospkg

from trash_utils.haversine_dist import haversine

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


def find_bounding_block(data, bbox, coord1, coord2):
	'''
	Given a current file, find the two bounding values from that file that the coord1 or 
		coord2 value falls between
	
	##TODO: GENERALIZE TO ANY/ALL THINGS
	'''
	all_currents = netcdf.NetCDFFile(current_file)

	lat = all_currents.variables['lat'][:].copy()
	lon = all_currents.variables['lon'][:].copy()
	lon -= 360

	min_lat = min(bbox[:, 0])
	max_lat = max(bbox[:, 0])
	min_lon = min(bbox[:, 1])
	max_lon = max(bbox[:, 1])

	width = haversine(bbox[0,0], bbox[0,1], bbox[1,0], bbox[1,1])
	height = haversine(bbox[1,0], bbox[1,1], bbox[2,0], bbox[2,1])

	lon_start = max(np.where(lon<=min_lon)[0])
	lon_end = min(np.where(lon>=max_lon)[0])

	#lon_start = np.searchsorted(lon, min_lon, 'left')
	#lon_end = np.searchsorted(lon, max_lon, 'right')

	lat_start = np.searchsorted(lat, min_lat, 'left')
	lat_end = np.searchsorted(lat, max_lat, 'right')

	if lon_start == lon_end:
		lon_idx = np.arange(lon_start-1, lon_end+1)
	else:
		lon_idx = np.array((lon_start, lon_end))
	if lat_start == lat_end:
		lat_idx = np.arange(lat_start-1, lat_end+1)
	else:
		lat_idx = np.array((lat_start, lat_end))

	print ("lon_idx: ", lon_idx)
	print ("lat_idx: ", lat_idx)

	# lon_idx = np.where((lon[:]>=min_lon) & (lon[:]<=max_lon))[0]
	# lat_idx = np.where((lat[:]>=min_lat) & (lat[:]<=max_lat))[0]

	lon_area = lon[lon_idx]
	lat_area = lat[lat_idx]

	print ("lon_area: ", lon_area)
	print ("lat_area: ", lat_area)

	pdb.set_trace()

	# lon_zeroed = lon_area - min(lon_area)
	# lon_ycoords = lon_zeroed*(height/max(lon_zeroed))*1000

	# lat_zeroed = lat_area - min(lat_area)
	# lat_xcoords = lat_zeroed*(width/max(lat_zeroed))*1000

	return lat_area, lon_area

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


def get_depth_block(bbox, topo_file):
	'''
	Input:
		bbox (np.ndarray): 4x2 array with lat/lon float coordinates for the area of interest
			Coordinates should be top left, top right, bottom right, bottom left
		topo_file (string): path to netcdf file containing the hydrodynamic data

	Returns:
		depth_map (np.ndarray): (3,n,m) array of the depth map (TODO: CHECK THIS IS CORRECT?)
	'''
	topo_depths = netcdf.NetCDFFile(depth_file)
	lat_coords = topo_depths.variables['lat'][:].copy()
	lon_coords = topo_depths.variables['lon'][:].copy()
	depths = topo_depths.variables['Band1'][:].copy()
	topo_depths.close()

	width = haversine(bbox[0,0], bbox[0,1], bbox[1,0], bbox[1,1])
	height = haversine(bbox[1,0], bbox[1,1], bbox[2,0], bbox[2,1])




	pass


def get_current_block(bbox, current_file):
	'''
	Input:
		bbox (np.ndarray): 4x2 array with lon/lat float coordinates for the area of interest
			Coordinates should be top left, top right, bottom right, bottom left
			Each bbox corner should be in [lon, lat] order
		current_file (string): path to netcdf file containing the hydrodynamic data

	Returns:
		u (np.ndarray): (n,m) array of all the currents in the u direction
		v (np.ndarray): (n,m) array of all the currents in the v direction
	'''
	all_currents = netcdf.NetCDFFile(current_file)

	lat = all_currents.variables['lat'][:].copy()
	lon = all_currents.variables['lon'][:].copy()
	lon -= 360
	u = all_currents.variables['u'][:].copy()
	v = all_currents.variables['v'][:].copy()
	all_currents.close()

	##find the bounds
	min_lat = min(bbox[:, 0])
	max_lat = max(bbox[:, 0])
	min_lon = min(bbox[:, 1])
	max_lon = max(bbox[:, 1])

	# ##Finding the lon bounds is different cause the values are negative
	# ##Not important todo: figure out a consistent method to pull both lat/lon
	# lon_start = max(np.where(lon<=min_lon)[0])
	# lon_end = min(np.where(lon>=max_lon)[0])
	# lat_start = np.searchsorted(lat, min_lat, 'left')
	# lat_end = np.searchsorted(lat, max_lat, 'right')

	# if lon_start == lon_end:
	# 	lon_idx = np.arange(lon_start-1, lon_end+1)
	# else:
	# 	lon_idx = np.array((lon_start, lon_end))
	# if lat_start == lat_end:
	# 	lat_idx = np.arange(lat_start-1, lat_end+1)
	# else:
	# 	lat_idx = np.array((lat_start, lat_end))

	# print ("lon_idx: ", lon_idx)
	# print ("lat_idx: ", lat_idx)

	##This is the unimportant todo fix
	##Processing cause the data is negative
	##Not sure this was worth the trouble
	lon_idx = find_bound_idx(lon, [min_lon, max_lon])
	lon_idx = np.sort(lon.shape[0]-1-lon_idx)

	lat_idx = find_bound_idx(lat, [min_lat, max_lat])

	print ("lon_idx: ", lon_idx)
	print ("lat_idx: ", lat_idx)

	##pull those coordinates from the current file
	# lon_area = lon[lon_idx]
	# lat_area = lat[lat_idx]

	print ('lon shape: ', lon.shape)
	print ('lat shape: ', lat.shape)
	print ("u shape: ", u.shape)

	u_area = u[0, :, lat_idx[0]:lat_idx[-1], lon_idx[0]:lon_idx[-1]]
	v_area = v[0, :, lat_idx[0]:lat_idx[-1], lon_idx[0]:lon_idx[-1]]

	print ('u_area shape: ', u_area.shape)
	print ("v_area shaep: ", v_area.shape)


	##Split into a different method

	##This is all interpolation things
	##allocate arrays for width, height
	width = haversine(bbox[0,0], bbox[0,1], bbox[1,0], bbox[1,1])*1000
	height = haversine(bbox[1,0], bbox[1,1], bbox[2,0], bbox[2,1])*1000
	
	uaxis = np.arange(0, u_area.shape[0])
	vaxis = np.arange(0, v_area.shape[0])

	print ('width: ', width)
	print ('uaxis: ', uaxis)

	xcoords = np.linspace(-width/2, width/2, width)
	ycoords = np.linspace(-height/2, height/2, height)
	
	pdb.set_trace()

	print ('xcoords: ', xcoords)
	print ('ycoords: ', ycoords)

	u_func = griddata(uaxis, u_area, ())
	v_func = interp1d(vaxis, v_area)

	##interpolate values across those arrays
	##u = interpolate
	##v = interpolate

	'''
	To test:
		pull the same coordinate from the smaller file
		pull the same coordinate from the larger current file
		Should be the same value

		maybe test how long it takes to run it?
	'''


	pass


def interpolate_pts(boundary1, boundary2, data_idx, boundary1_data, boundary2_data):
	'''
	Input:
		boundary1 (float): any

	'''


	print "FOUND THIS METHOD!"
	pass


rospack = rospkg.RosPack()
trash_finder_path = rospack.get_path("trash_finder")
data_path = rospack.get_path("data_files")

config = SafeConfigParser()
config.read(trash_finder_path+'/scripts/demo_configs.ini')

currents1 = config.get('CURRENTS', 'current_file')
currents_path = data_path+'/'+currents1

places = np.array([
	[config.get('COORDS', 'loc1_lat'), config.get('COORDS', 'loc1_lon')],
	[config.get('COORDS', 'loc2_lat'), config.get('COORDS', 'loc2_lon')],
	[config.get('COORDS', 'loc3_lat'), config.get('COORDS', 'loc3_lon')],
	[config.get('COORDS', 'loc4_lat'), config.get('COORDS', 'loc4_lon')]
	], dtype=float)
print ("PLACES: ", places)
get_current_block(places, currents_path)
