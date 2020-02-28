'''
Goal: discretizes the map into a lawn mower pattern in the direction of interest
	Takes the shortest dimension (which means the rows will be long. ie less turns)
	Calculates the waypoints needed to complete the pattern


'''

import numpy as np
##Allow for polygons
import shapely.geometry as sh

##Global decomposition is difficult. Do we need to calculate the gradient of the lines?
##For now, assume the given bbox is rectangular

global_bbox = sh.Polygon([[0,0], [155,0], [155,100], [0,155]])
def global_decomp(bbox):
	minx, miny, maxx, maxy = global_bbox.bounds
	g_height = maxy - miny
	g_width = maxx - minx

	search_radius = 103
	search_width = 500

	##Break arbitrarily into solid boxes
	print (g_height//search_radius)
	print (g_width//search_width)

	if g_height//search_radius <= 0:
		x_bbox = np.array([minx, maxx])
	else:
		x_bbox = np.linspace(minx, maxx, (g_height//search_radius)+2)

	if g_width//search_width <= 0:
		y_bbox = np.array([miny, maxy])
	else:
		y_bbox = np.linspace(miny, maxy, (g_width//search_width)+2)

	print ("x_bbox: ", x_bbox)
	print ("y_bbox: ", y_bbox)

	##Figure out all the bbox waypoints:
	all_bbox = np.zeros((len(x_bbox)-1*len(y_bbox)-1, 4))
	



global_decomp(global_bbox)
	# ##Take the longest direction
	# ##Arbitrarily: if we want to minimize the number of turns, then we want to keep the longest dimension
	# ## and cut across the shortest dimension
	# ##Also cut the long dimension by some arbitrary length

	# height_rows = g_height/search_radius
	# width_rows = g_width/search_radius

	# if height_rows > width_rows:
	# 	height_pts = np.arange(0, g_height + height_rows, search_radius)
	# 	width_pts = np.arange(0, g_width+longest_search_length, longest_search_length)
	# 	create_waypoints(height_pts, width_pts)

	# else:
	# 	height_pts = np.arange(0, g_height + longest_search_length, longest_search_length)
	# 	width_pts = np.arange(0, g_width + width_rows, search_radius)
	# 	create_waypoints(width_pts, height_pts)

##orient it in the correct direction
# rot_global_dim = 

##Create the waypoints, starting from the bottom left side
def create_waypoints(long_side_arr, short_side_arr):
	print ("len long_side_arr: ", len(long_side_arr))
	waypoints = np.zeros((2, len(long_side_arr)*2))
	# waypoints[1][0] = short_side_arr[0]


	long_idx = np.arange(0, len(long_side_arr)*2, 2)
	long_idx_plus = long_idx + 1

	print ("long_idx: ", long_idx)
	print ("long_idx_plus: ", long_idx_plus)
	waypoints[1][long_idx] = long_side_arr
	waypoints[1][long_idx_plus] = long_side_arr


	##TODO: Will need to figure out how to incorporate multiple boxes
	##Convert this to a for loop:




	short_idx = np.arange(1, len(long_side_arr)*2, 4)
	short_idx_ext = short_idx + 1
	short_idx_ext = short_idx_ext[:-1]

	short_idx_plus = np.arange(3, len(long_side_arr)*2-1, 4)
	short_idx_plus_ext = short_idx_plus + 1

	print (short_side_arr)
	print ("short_idx: ", short_idx)
	print ("short_idx: ", short_idx_ext)
	print ("short_idx_plus: ", short_idx_plus)
	print ("short_idx_plus: ", short_idx_plus_ext)

	waypoints[0][short_idx] = short_side_arr[1]
	waypoints[0][short_idx_ext] = short_side_arr[1]
	waypoints[0][short_idx_plus] = short_side_arr[0]
	waypoints[0][short_idx_plus+1] = short_side_arr[0]

	print (waypoints)


	# print ("waypoints starting arr: \n", waypoints)
	# print ("long_side_arr: \n", long_side_arr)
	# print ("short_side_arr: \n", short_side_arr)
	# long_counter = 0
	# short_counter = 1
	# for i in range(len(long_side_arr)):
	# 	print ("long_counter: \n", long_counter)
	# 	print ("short_counter: \n", short_counter	)

	# 	waypoints[0][long_counter:long_counter+2] = long_side_arr[i]
	# 	waypoints[1][short_counter:short_counter+2] = short_side_arr[i]

	# 	print ("waypoints: \n", waypoints)

	# 	long_counter += 2
	# 	short_counter += 2

	return waypoints



'''
	Make sure here that the boxes are odd rows with odd number of boxes
'''


def calc_lawn_mower_waypoints(self, bbox, search_radius):
	'''
	Input:
	bbox = [upper left, upper right, bottom right, bottom left coords]
	search_radius = height of the search rows
	overlap = percentage of each search_radius that needs to overlap

	Generic mowing the lawn algorithm

	'''

