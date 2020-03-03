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

from Topological_Map import *

##Global decomposition is difficult. Do we need to calculate the gradient of the lines?
##For now, assume the given bbox is rectangular

class Mowing_Lawn_BBox_Search:

	def __init__(self, global_bbox):
		self.global_bbox_height_max = 0.05
		self.global_bbox_width_max = 0.05

		self.search_radius = 0.01
		self.search_width = 0.01
		self.global_bbox = global_bbox

	def global_decomp(self):
		##Break down the global bbox into smaller more managable boxes
		x_bbox, y_bbox = self.calc_bbox_lines(self.global_bbox, self.global_bbox_height_max, self.global_bbox_width_max)

		print ("x_bbox: ", len(x_bbox))
		print ("y_bbox: ", len(y_bbox))


		##This is time consuming for many points. Get rid of the two for loops
		all_bbox = []
		np_all_bbox = []
		for i in range(x_bbox.shape[0]-1):
			for j in range(y_bbox.shape[0]-1):
				bbox = sh.Polygon([ [x_bbox[i], y_bbox[j]], [x_bbox[i+1], y_bbox[j]], [x_bbox[i+1], y_bbox[j+1]], [x_bbox[i], y_bbox[j+1]], [x_bbox[i], y_bbox[j]] ])
				all_bbox.append(bbox)

				np_all_bbox.append([[x_bbox[i], y_bbox[j]], [x_bbox[i+1], y_bbox[j]], [x_bbox[i+1], y_bbox[j+1]], [x_bbox[i], y_bbox[j+1]], [x_bbox[i], y_bbox[j]] ])

		np_all_bbox = np.array(np_all_bbox).reshape((-1,2))
		##TODO: Visualize all the bbox here:
		# print (all_bbox)
		print (np_all_bbox)

		plt.plot(npta[:,0], npta[:,1])
		plt.plot(np_all_bbox[:,0], np_all_bbox[:,1])
		plt.show()


		##Calculate waypoints for each of the smaller bbox areas
		##for each bbox in all_bbox:
			##Rotate by some amount
			##Calculate the waypoints for each
			##Calculate the cost of traveling that path
			##Keep the best cost and rotation
		
		##TODO: Visualize the first bbox
		all_bbox0_np = np.array(all_bbox[0].exterior.coords)

		best_cost = 99999999999999999999

		rotation = 45
		rotated_bbox = sa.rotate(all_bbox[0], 10) ##By default, takes in rotation by degrees
		print ("all_bbox[0]: ")
		print(all_bbox[0])

		print ("rotated bbox: ")
		print (rotated_bbox)

		rotated0_np = np.array(rotated_bbox.exterior.coords)

		pdb.set_trace()
		x_bbox0, y_bbox0 = self.calc_bbox_lines(rotated_bbox, self.search_radius, self.search_width)
		print ("x_bbox0: ", x_bbox0)
		print ("y_bbox0: ", y_bbox0)

		waypts = self.calc_mowing_lawn(rotated_bbox, y_bbox0, start_end="left")

		pdb.set_trace()

		plt.plot(npta[:,0], npta[:,1])
		plt.plot(rotated0_np[:,0], rotated0_np[:,1])
		plt.plot(waypts[:,0], waypts[:,1])
		plt.show()

		##Calculate the cost of traveling that path:
		##Use shortest path. Swap out for Haversine later
		cost = self.calc_path_dist(waypts)

		if cost < best_cost:
			best_cost = cost
			best_angle = rotation


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
		# print ("all lines:")
		# print(all_lines)

		# intersections = bbox.intersection(all_lines)
		intersections = all_lines.intersection(bbox)
		# print ("intersections: ")
		# print (intersections)

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
			
		# print ("waypoints: ", waypoints)
		# pdb.set_trace()		

		return np.array(waypoints)

	##Would we need to correct for earth distance here?
	##Use haversine instead
	def calc_path_dist(self, waypts):
		total_dist = 0
		print (waypts.shape[0])
		for pt in range(waypts.shape[0]):
			total_dist += abs(np.linalg.norm(waypts[pt] - waypts[pt+1]))
		print ("total_dist: ", total_dist)

		return total_dist

if __name__ == '__main__':
	# global_bbox = sh.Polygon([[0,0], [155,0], [155,100], [0,155]])

	TM = Topological_Map()

	map_contours = TM.get_map_contours()
	# for i in range(1, len(map_contours)):
	print ("number of areas: ", len(map_contours))
	area1 = map_contours[2].get_paths()[0].vertices
	area2 = map_contours[3].get_paths()[1].vertices


	plt.show()
	# plt.clf()
	plt.plot(area1[:,0], area1[:,1], 'k')
	plt.plot(area2[:,0], area2[:,1])
	plt.show()

	pdb.set_trace()

	##Connect them into a polygon now:
	area12 = np.vstack((area1, np.flipud(area2)))
	test_area = sh.Polygon(area12)
	npta = np.array(test_area.exterior.coords)

	plt.plot(npta[:,0], npta[:,1])
	plt.show()

	pdb.set_trace()

	M = Mowing_Lawn_BBox_Search(test_area)
	M.global_decomp()


	# main()
