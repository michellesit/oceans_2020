from math import atan2, sin, cos

import numpy as np
import shapely.geometry as sg
import shapely.affinity as sa
import matplotlib.pyplot as plt

'''
All of the complete coverage algorithms to be shared across files

'''


def calc_mowing_lawn(bbox, y_bbox, start_end='right'):
	'''
	Calculate the intersection of the lines with the bbox
	Orders the waypoints according to either left/right start/end

	Input:
		bbox (sh.Polygon) : some polygon to calculate the mowing lawn pattern
		y_bbox (np.ndarray) : Array of heights that the lines are calculated at
							Each point in this array corresponds to a line.
		start_end (str) : "right" or "left". Determines which side the first
						waypoint comes from. This determines the ordering of all
						the following waypoints.

	Output:
		waypoints (np.ndarray) : ordered waypoints on how to traverse this bbox
	'''
	all_intersections = []
	minx, miny, maxx, maxy = bbox.bounds
	for i in range(y_bbox.shape[0]):
		lines = sg.LineString([ [minx-abs(minx*0.4), y_bbox[i]],
								[maxx+abs(maxx*0.4), y_bbox[i]] ])
		intersections = bbox.intersection(lines)

		if intersections.is_empty == False:
			##TODO: sort the intersections so they are always left to right
			# print (zip(lines.xy[0], lines.xy[1]))
			# ziplines = zip(lines.xy[0], lines.xy[1])
			# npint = np.array(intersections)

			# plt.plot(bbox.exterior.coords.xy[0], bbox.exterior.coords.xy[1])
			# plt.plot(lines.xy[0], lines.xy[1])
			# if npint.ndim == 1:
			# 	plt.scatter(npint[0], npint[1])
			# else:
			# 	plt.scatter(npint[:,0], npint[:,1])
			# plt.show()
			# pdb.set_trace()

			all_intersections.append(np.array(intersections))

	##TODO: Should add in a check here to make sure there are intersecting lines

	##order the waypoints accordingly
	waypoints = []
	for i in range(len(all_intersections)):
		line_pts = all_intersections[i]

		if all_intersections[i].ndim == 1:
			waypoints.extend((line_pts, line_pts))
			continue

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

	return np.array(waypoints)

'''
def simple_cc(search_bbox):
	rotation = np.arange(0, 360, 10)
	best_cost = 99999999999999999;
	for r in range(len(rotation)):
		rotated_bbox = sa.rotate(search_bbox, rotation[r]) ##Default, takes in rotation by degrees
		waypts = calc_mowing_lawn(rotated_bbox, y_bbox0, start_end="left")
		centered_waypoints = sa.rotate(sh.Polygon(waypts), -rotation[r])
		np_centered_waypoints = np.array(zip(centered_waypoints.exterior.coords.xy[0], 
								 centered_waypoints.exterior.coords.xy[1]))
		cost = self.est_propulsion(np_centered_waypoints, best_cost, 
										est_uuv_pos=False, use_bound=True, 
										visualize=False)

		if cost < best_cost:
			best_cost = cost
			best_waypoints = np_centered_waypoints
			best_rotation = rotation[r]
			# best_time = cost

	return best_cost
'''

# def est_propulsion(waypoints, bound_cost, est_uuv_pos=True, use_bound=True, visualize=False):
'''
	For each waypoint in the path, iteratively calculate how long it would take to get there.
	Do some vector math to calculate what direction/where you'd end up based 
		on currents and the uuv's vector heading/power

	Visualiztion shows an animation of the UUV's progress moving toward each waypoint

	Calculates for each 1 second (since we are traveling at 5knots = 2.5722m/sec)
	uuv_speed = constant for now

	Inputs:
		waypoints (np.ndarray) : Ordered waypoints that the UUV needs to take.
		bound_cost (int) : Branch and bound optimization. The max cost that this method may take.
		est_uuv_pos (bool) : True = Sets uuv position to be the first waypoint in the given waypoints.
							False, uses the last known position of the uuv (self.uuv_position)
		use_bound (bool) : Branch and bound optimization. If the timestep value is greater than
							bound_cost, then the method immediately terminates and returns an
							unreasonable cost.
		visualize (bool) : Shows an animation of the UUV progressing toward each waypoint at each step
	Returns:
		timesteps (int) : number of timesteps it took to traverse all these waypoints

'''

'''
	##Set uuv position to be the first waypoint
	##Otherwise, use the uuv's current position as its starting location
	if est_uuv_pos:
		self.uuv_position = waypoints[1]

	##Visualizes the path that will be taken
	if visualize:
		plt.subplots(111)
		ax = plt.subplot(111)
		plt.plot(waypoints[:,0], waypoints[:,1])
	
	timesteps = 0
	for wp in range(len(waypoints)):
		goal = waypoints[wp]

		while abs(np.linalg.norm(self.uuv_position - goal)) > self.goal_threshold:
			##Calculate the heading to that location
			desired_heading = atan2(goal[1] - self.uuv_position[1], goal[0] - self.uuv_position[0])
			goal_u = cos(desired_heading) * np.linalg.norm(goal - self.uuv_position)
			goal_v = sin(desired_heading) * np.linalg.norm(goal - self.uuv_position)
			current_time_hrs = (timesteps + self.global_timesteps)/3600

			z = abs(dfunc(self.uuv_position))
			current_u = ufunc([current_time_hrs, z, self.uuv_position[0], self.uuv_position[1]])[0]
			current_v = vfunc([current_time_hrs, z, self.uuv_position[0], self.uuv_position[1]])[0]
			# current_heading = atan2(current_v, current_u)

			##vector math
			desired_u = goal_u + current_u
			desired_v = goal_v - current_v
			resulting_heading = atan2(desired_v, desired_u)

			uuv_u = cos(resulting_heading) * self.uuv_speed
			uuv_v = sin(resulting_heading) * self.uuv_speed

			resulting_speed = np.array([uuv_u + current_u, uuv_v + current_v])

			##TODO: What does each Circle represent?
			##TODO: What do each arrow represents?
			if visualize:
				ax.quiver(self.uuv_position[0], self.uuv_position[1], uuv_u, uuv_v, scale=1, scale_units='xy', color='green') ##uuv
				wp_c = plt.Circle((goal[0], goal[1]), 1, color='black') ##waypoint
				ax.add_artist(wp_c)
				uuv_end = plt.Circle((self.uuv_position[0] + resulting_speed[0], self.uuv_position[1] + resulting_speed[1]), 1, color="blue")
				ax.add_artist(uuv_end)
				plt.pause(0.05)

			self.uuv_position += resulting_speed
			self.uuv_heading = resulting_heading

			##add to timestep count (time)
			timesteps += 1
			if use_bound and timesteps > bound_cost:
				return 99999999999999999

	return timesteps

'''