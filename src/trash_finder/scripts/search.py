'''
Goal: discretizes the map into a lawn mower pattern in the direction of interest
	Takes the shortest dimension (which means the rows will be long. ie less turns)
	Calculates the waypoints needed to complete the pattern


'''

import numpy as np
import shapely.geometry as sh

import pdb

##Global decomposition is difficult. Do we need to calculate the gradient of the lines?
##For now, assume the given bbox is rectangular

class Mowing_Lawn_BBox_Search():

	def __init__(self):
		self.global_bbox_height_max = 50
		self.global_bbox_width_max = 500

		self.search_radius = 10
		self.search_width = 500
		self.global_bbox = global_bbox

	def global_decomp(self, bbox):
		# minx, miny, maxx, maxy = global_bbox.bounds
		# g_height = maxy - miny
		# g_width = maxx - minx

		# # self.search_radius = 10
		# # self.search_width = 500

		# ##Break arbitrarily into solid boxes
		# print (g_height//self.search_radius)
		# print (g_width//self.search_width)

		# if g_height//self.search_radius <= 0:
		# 	x_bbox = np.array([minx, maxx])
		# else:
		# 	x_bbox = np.linspace(minx, maxx, (g_height//self.search_radius)+2)

		# if g_width//self.search_width <= 0:
		# 	y_bbox = np.array([miny, maxy])
		# else:
		# 	y_bbox = np.linspace(miny, maxy, (g_width//self.search_width)+2)

		x_bbox, y_bbox = self.calc_bbox_lines(global_bbox, self.global_bbox_height_max, self.global_bbox_width_max)

		print ("x_bbox: ", x_bbox)
		print ("y_bbox: ", y_bbox)

		##Figure out all the bbox waypoints:
		##Append all the polygon waypoints to the list
		all_bbox = []
		for i in range(x_bbox.shape[0]-1):
			for j in range(y_bbox.shape[0]-1):
				bbox = sh.Polygon([ [x_bbox[i], y_bbox[j]], [x_bbox[i+1], y_bbox[j]], [x_bbox[i+1], y_bbox[j+1]], [x_bbox[i], y_bbox[j+1]] ])
				all_bbox.append(bbox)

		# print (all_bbox)


		##Calculate waypoints for each bbox
		##for each bbox in all_bbox:
			##Calculate the waypoints for each
		print ("all_bbox[0]: ")
		print(all_bbox[0])
		x_bbox0, y_bbox0 = self.calc_bbox_lines(all_bbox[0], self.search_radius, self.search_width)
		print ("x_bbox0: ", x_bbox0)
		print ("y_bbox0: ", y_bbox0)

		self.calc_mowing_lawn(all_bbox[0], x_bbox0)


		# ##Take the longest direction
		# ##Arbitrarily: if we want to minimize the number of turns, then we want to keep the longest dimension
		# ## and cut across the shortest dimension
		# ##Also cut the long dimension by some arbitrary length

		# height_rows = g_height/self.search_radius
		# width_rows = g_width/self.search_radius

		# if height_rows > width_rows:
		# 	height_pts = np.arange(0, g_height + height_rows, self.search_radius)
		# 	width_pts = np.arange(0, g_width+longest_search_length, longest_search_length)
		# 	create_waypoints(height_pts, width_pts)

		# else:
		# 	height_pts = np.arange(0, g_height + longest_search_length, longest_search_length)
		# 	width_pts = np.arange(0, g_width + width_rows, self.search_radius)
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


	def calc_bbox_lines(self, bbox, height_radius, width_radius):
		'''
		Input:
		bbox = Polygon([upper left, upper right, bottom right, bottom left coords])
		self.search_radius = height of the search rows

		Output:
		x_bbox, y_bbox = 

		'''
		minx, miny, maxx, maxy = bbox.bounds
		bbox_height = maxy - miny
		bbox_width = maxx - minx

		num_height_lines = bbox_height//height_radius
		num_width_lines = bbox_width//width_radius

		print (num_height_lines)
		print (num_width_lines)

		if num_height_lines <= 0:
			x_bbox = np.array([minx, maxx])
		else:
			x_bbox = np.linspace(minx, maxx, (num_height_lines)+2)

		if num_width_lines <= 0:
			y_bbox = np.array([miny, maxy])
		else:
			y_bbox = np.linspace(miny, maxy, (num_width_lines)+2)

		return x_bbox, y_bbox


	def calc_mowing_lawn(self, bbox, x_bbox):
		##Calculate the intersection of the lines with the bbox:
		##For each pair of x_bbox lines, calculate intersection with bbox
		print(bbox)
		
		#lines = np.zeros((x_bbox.shape[0], 2, 2))
		#lines[:, 0, 0] = bbox.bounds[0]
		#lines[:, 1, 0] = bbox.bounds[2]
		#lines[:, 0, 1] = x_bbox
		#lines[:, 1, 1] = x_bbox
			
		#lines = lines.reshape((-1, 2))
		#print ('lines: ', lines)
		start_end = "left"

		all_lines = []
		for i in range(x_bbox.shape[0]):
			lines = sh.LineString([ [bbox.bounds[0], x_bbox[i]], [bbox.bounds[2], x_bbox[i]] ])
			all_lines.append(lines)

		all_lines = sh.MultiLineString(all_lines)

		intersections = bbox.intersection(all_lines)
		print ("intersections: ")
		print (intersections)

		##append that to to the bbox waypts
		waypoints = []
		for i in range(len(intersections)):
			line_pts = intersections[i].coords		
		
			print ("i: ", i)	
			print ('i%2: ', i%2)
			if start_end == "right":
				if i%2 == 0:
					waypoints.extend((line_pts[1], line_pts[0]))
					#waypoints.append(line_pts[0])
				else:
					waypoints.extend((line_pts[0], line_pts[1]))

			elif start_end == "left":
				if i%2 == 0:
					waypoints.extend((line_pts[0], line_pts[1]))
				else:
					waypoints.extend((line_pts[1], line_pts[0]))
			
			print ("waypoints: ", waypoints)
			pdb.set_trace()		

		##order the waypoints accordingly


	def rotate_bbox(self):
		'''
		rotate both the bbox and lines
		'''


if __name__ == '__main__':
	global_bbox = sh.Polygon([[0,0], [155,0], [155,100], [0,155]])
	M = Mowing_Lawn_BBox_Search()

	M.global_decomp(global_bbox)

	# main()
