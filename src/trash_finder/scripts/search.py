'''
Goal: discretizes the map into a lawn mower pattern in the direction of interest
	Takes the shortest dimension (which means the rows will be long. ie less turns)
	Calculates the waypoints needed to complete the pattern


'''

import numpy as np
import shapely.geometry as sh
import shapely.affinity as sa

import pdb
import matplotlib.pyplot as plt

##Global decomposition is difficult. Do we need to calculate the gradient of the lines?
##For now, assume the given bbox is rectangular

class Mowing_Lawn_BBox_Search():

	def __init__(self):
		self.global_bbox_height_max = 50
		self.global_bbox_width_max = 50

		self.search_radius = 10
		self.search_width = 10
		self.global_bbox = global_bbox

	def global_decomp(self, bbox):
		##Break down the global bbox into smaller more managable boxes
		x_bbox, y_bbox = self.calc_bbox_lines(global_bbox, self.global_bbox_height_max, self.global_bbox_width_max)

		print ("x_bbox: ", x_bbox)
		print ("y_bbox: ", y_bbox)

		all_bbox = []
		for i in range(x_bbox.shape[0]-1):
			for j in range(y_bbox.shape[0]-1):
				bbox = sh.Polygon([ [x_bbox[i], y_bbox[j]], [x_bbox[i+1], y_bbox[j]], [x_bbox[i+1], y_bbox[j+1]], [x_bbox[i], y_bbox[j+1]] ])
				all_bbox.append(bbox)


		##Calculate waypoints for each of the smaller bbox areas
		##for each bbox in all_bbox:
			##Rotate by some amount
			##Calculate the waypoints for each
			##Calculate the cost of traveling that path
			##Keep the best cost and rotation

		rotation = 10
		rotated_bbox = sa.rotate(all_bbox[0], 10) ##By default, takes in rotation by degrees
		print ("all_bbox[0]: ")
		print(all_bbox[0])

		print ("rotated bbox: ")
		print (rotated_bbox)

		pdb.set_trace()
		x_bbox0, y_bbox0 = self.calc_bbox_lines(rotated_bbox, self.search_radius, self.search_width)
		print ("x_bbox0: ", x_bbox0)
		print ("y_bbox0: ", y_bbox0)

		waypts = self.calc_mowing_lawn(all_bbox[0], y_bbox0, start_end="left")

		##Calculate the cost of traveling that path:
		##TODO


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
		bbox_height = maxy - miny
		bbox_width = maxx - minx

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
		##Calculate the intersection of the lines with the bbox:
		##For each pair of x_bbox lines, calculate intersection with bbox

		all_lines = []
		for i in range(y_bbox.shape[0]):
			lines = sh.LineString([ [bbox.bounds[0]-(bbox.bounds[0]*0.4), y_bbox[i]], [bbox.bounds[2]+(bbox.bounds[2]*0.4), y_bbox[i]] ])
			all_lines.append(lines)

		##TODO: Should add in a check here to make sure there are intersecting lines
		all_lines = sh.MultiLineString(all_lines)
		print ("all lines:")
		print(all_lines)

		# intersections = bbox.intersection(all_lines)
		intersections = all_lines.intersection(bbox)
		print ("intersections: ")
		print (intersections)

		##TODO: add in logic to make sure the coordinates make sense?
		##order the waypoints accordingly
		waypoints = []
		for i in range(len(intersections)):
			line_pts = intersections[i].coords		

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
			
		print ("waypoints: ", waypoints)
		pdb.set_trace()		

		return waypoints


if __name__ == '__main__':
	global_bbox = sh.Polygon([[0,0], [155,0], [155,100], [0,155]])
	M = Mowing_Lawn_BBox_Search()

	M.global_decomp(global_bbox)

	# main()
