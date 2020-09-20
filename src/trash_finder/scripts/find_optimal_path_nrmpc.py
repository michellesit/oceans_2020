from collections import OrderedDict
from numpy.linalg import norm
import numpy as np

def find_optimal_path_nrmpc(time_start, epoch, start_pos, end_pos, max_current):
	'''
	Following the paper
	Predictive motion planning for AUVs subject to strong time-varying currents and forecasting uncertainties (Huynh, Van T., Dunbabin, Matthew, Smith, Ryan N.) (2015)

	They use a Nonlinear Robust Model Predictive Control (NRMPC) algorithm to define their system's state and control params. Then use A* to explore the space and control params until they get to their final destination.

	Input:
		time_start = time in seconds
		epoch = how many steps into the future to calculate
		start_pos = (x,y,depth)
		end_pos = (x,y,depth)
		max_current (array) = [max u current from the whole map, max v current from the whole map]

	Returns:
		middle_pts = all the waypoints (x,y,depth) to get to the end_pos
		middle_time = all the times it would take to get to end_pos
		ending_control = ending speed in (u,v)

	'''
	##parameters?
	E_appr = 5 ##Approximate, minimum energy per sampling time consumed by the vehicle at each time step
	max_current = [1.2, 1.4]
	uuv_speed = 2.5722

	##Do sorted dictionary to hold the cost, heuristic_cost, and total_cost values
	all_map = {}

	##init starting node
	##heuristic_cost = euclidean_dist(current_pos, end_pos)/max(euclidean_dist(v_current + v_AUV_relative_speed)) * E_appr
	##heuristic cost is taken from the paper
	heuristic_cost = ( norm(start_pos, end_pos)/max(norm(max_current + uuv_speed)) )*E_appr
	all_map[start_pos] = ['cost': 0, 'heuristic_cost': heuristic_cost, 'total_cost': heuristic_cost]

	visited_list = []
	to_be_visited_list = [] ##This will just be the sorted dict keys list

	##Find the possible positions around the goal
	##for each heading, calculate where the AUV would end up
	base_h_angle = 45
	num_headings = 360/base_h_angle

	current_pos = start_pos
	for h_angle in range(num_headings):
		desired_heading = base_h_angle*h_angle
		goal_u = cos(desired_heading) * 50 + current_pos[0]
		goal_v = sin(desired_heading) * 50 + current_pos[1]
		current_time_hrs = (time_start + self.global_timesteps)/3600

		z = start_pos[-1]
		current_u = self.ufunc([current_time_hrs, z, current_pos[0], current_pos[1]])
		current_v = self.vfunc([current_time_hrs, z, current_pos[0], current_pos[1]])

		##vector math
		desired_u = goal_u + current_u
		desired_v = goal_v + current_v
		resulting_heading = atan2(desired_v, desired_u)

		uuv_u = cos(resulting_heading) * uuv_speed
		uuv_v = sin(resulting_heading) * uuv_speed
		resulting_speed = [uuv_u + current_u, uuv_v + current_v]

		
		A_eta = np.eye(2)*np.array([current_pos[0], current_pos[1]]).transpose()
		B = np.array([[cos(resulting_heading), 0, 1, 0], [0, sin(resulting_heading), 0, 1]])
		u = np.array([uuv_u, uuv_v, current_u, current_v]).reshape((-1,1))
		cost = A_eta + B*u

		print ('cost: ', cost)

		heuristic_cost = ( norm(current_pos, end_pos)/max(norm(max_current + uuv_speed)) )*E_appr
		print ('heuristic_cost: ', heuristic_cost)



	##Also calculate the cost of getting to that node
	##Also calculate the heuristic cost for each of the nodes
	##

	##Sort the nodes based on their total cost
	##Append the current node to the visited list
	##Take the lowest cost node to be the next node
	##Append 
	current_node = sorted_nodes[0]


	

	goal = 

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