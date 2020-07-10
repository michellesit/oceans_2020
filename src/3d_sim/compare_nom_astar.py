from math import atan2, sin, cos, acos

import numpy as np
import shapely.geometry as sg
from shapely.ops import nearest_points
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from trash_utils.Env import Env
from trash_utils.UUV import UUV
from trash_utils.finder_utils import (grid_data)
from trash_utils.trash_lib import ( init_hotspots, 
                                    visualize_trash_flow,
                                    update_trash_pos,
                                    visualize_trash_step)
from trash_utils.cc_utils import calc_mowing_lawn, search_for_trash
from trash_utils.fourD_utils import cost_to_waypoint_v1, follow_path_waypoints

from hs4dpp_main_utils import calc_heuristic_denom, calc_all_headings
from hs4dpp_path_utils import calc_ball_cost, find_optimal_path_nrmpc

import pdb

'''
Does both nominal and 4D A* solution to compare results

'''

class Compare_Nom_AStar():

	def __init__(self):
		self.env = Env() ##This contains (ufunc, vfunc, width, height, u_boost, v_boost)
        self.num_hotspots = 6

        self.uuv = UUV()
        
        #For 4D path planning
        self.desired_speed = 2.5722     ##meters/second (5 knots)
        self.astar_goal_dist = 20
        self.uuv_end_threshold = 10
        self.max_num_epochs = 15000

        self.nom_goal_dist = 50

        self.heuristic_denom = calc_heuristic_denom(self.env, self.desired_speed)
        self.base_h_angle = 45


    def compare(self, ball_cart_pos, path_order):
    	'''
        Calculates the paths to compare to the nominal solution by:
        - Calculating closest points between two hotspots
        - Calling find_optimal_path_nrmpc to find the path between the two points

        Path to cover each hotspot is a mowing the lawn pattern

        Inputs:
            hotspot_dict (Dict)        : key = hotspot id number (1,2,..., num_hotspots)
                                        values = latest pos of all the trash in the hotspot
            wpt_spacing (int)          : distance (meters) between the nominal waypoints
            heuristic_denom (float)    :  
            ball_cart_pos (np.ndarray) : 
            path_order (np.ndarray)    : 

        Returns:
            cc_paths (np.ndarray)  : waypoints to do complete coverage of the hotspot
            all_paths (np.ndarray) : Nominal waypoints to travel from hotspot to hotspot
                                     To access the path from hotspot 1 to 4 for example:
                                     all_paths[1][4]
            final_path (np.ndarray) : 

        TODO: handle return values here
        '''

        ##For each of these hotspots, 
        ##Calculate the path to each other 
        ## and the complete coverage algorithm to cover them
        ##TODO: update this with some other data structure/add the ones for keeping track of costs

        final_path = []
        total_energy_cost = 0.0
        total_time_sec = 0.0
        total_paper_cost = 0.0

        last_trash_update_sec = 0.0
        ##Calculate the paths in the order given
        ##TODO: ADD IN NOM PATH
        ##TODO: Visualize both paths
        for p_idx in range(len(path_order)):

            ##Update the trash positions up to this time
            if total_time_sec > 0:
                for diff_sec in range(last_trash_update_sec, total_time_sec):
                    diff_hrs = diff_sec/3600
                    hotspot_dict = update_trash_pos(hotspot_dict, diff_hrs, self.env)
                last_trash_update_sec = total_time_sec


            ## Get convex hull of each hotspot plus some buffer
            convexhull_a = sg.MultiPoint(hotspot_dict[path_order[p_idx]][-1][:, 0:2]).convex_hull
            buffer_a = convexhull_a.buffer(5)
            ax, ay = buffer_a.exterior.coords.xy
            [minx, miny, maxx, maxy] = buffer_a.bounds
            cc_y_lines = np.arange(miny, maxy, 5)

            ##Calculate the CC pattern on this hotspot
            cc_wpts = calc_mowing_lawn(buffer_a, cc_y_lines)
            cc_depths = self.env.dfunc(cc_wpts)
            cc_wpts = np.hstack((cc_wpts, cc_depths.reshape(-1,1)))
            ##Calculate the cost of traveling this cc_path at this time
            energy_cost, time_cost_sec, est_cost = search_for_trash(cc_wpts, 
                                                                    hotspot_dict, 
                                                                    self.uuv, 
                                                                    self.env, 
                                                                    self.desired_speed, 
                                                                    *[False])

            total_energy_cost += energy_cost
            total_time_sec += time_cost_sec
            total_paper_cost += est_cost
            final_path.extend([cc_wpts])

            print ("total energy  : ", total_energy_cost)
            print ("total time (s): ", total_time_sec)
            print ("total cost eq : ", total_paper_cost)
            print ("final_path    : ", final_path)

            convexhull_b = sg.MultiPoint(hotspot_dict[path_order[p_idx+1]][-1][:, 0:2]).convex_hull
            buffer_b = convexhull_b.buffer(5)
            bx, by = buffer_b.exterior.coords.xy

            ##Calculate the closest points between the hotspots
            pt1, pt2 = nearest_points(buffer_a, buffer_b)
            pt1_depth = self.env.dfunc([pt1.x, pt1.y])[0]
            pt2_depth = self.env.dfunc([pt2.x, pt2.y])[0]
            pt1_3d = np.array([pt1.x, pt1.y, pt1_depth])
            pt2_3d = np.array([pt2.x, pt2.y, pt2_depth])

            ##TODO: handle time_arr thing and appropriate costs
            astr_eq_cost, astr_time_cost_sec, astr_energy_cost, astr_path = find_optimal_path_nrmpc(total_time_sec, pt1_3d, pt2_3d, ball_cart_pos, self.uuv, self.env, self.heuristic_denom, self.desired_speed, self.goal_dist,
                self.uuv_end_threshold, self.max_num_epochs, *[True])

            ##Add these costs to the total costs
            total_energy_cost += astr_energy_cost
            total_time_sec += astr_time_cost_sec
            total_paper_cost += astr_eq_cost
            final_path.extend([astr_path])

            print ("total energy  : ", total_energy_cost)
            print ("total time (s): ", total_time_sec)
            print ("total cost eq : ", total_paper_cost)
            print ("final_path    : ", final_path)            

            pdb.set_trace()


    def main(self):
    	## Create hotspots of trash and (optinal) visualize
        trash_x_centers = np.array([-250.98494701, -504.8406451, \
                                    -132, 345, 876, 423]).reshape(-1,1)
        trash_y_centers = np.array([-508.96243035, -877.89326774, \
                                    -687, 354, 120, 348]).reshape(-1,1)
        ##Can specify trash distribution covariance for all hotspots:
        # trash_sigma = [[0.5, 0], [0, 0.1]]
        ##Use this to auto generate gaussian trash distributions:
        trash_sigma = [[], []]               
        hotspot_dict = init_hotspots(trash_x_centers, trash_y_centers,
                                     trash_sigma, 20, self.env.dfunc)

        for xx in range(1000):
            hotspot_dict = update_trash_pos(hotspot_dict, 0, self.env)


        ball_cart_pos, ball_sphere_headings = calc_all_headings(self.base_h_angle)
		hotspot_order = [0, 1, 2, 3, 4, 5]

		compare(ball_cart_pos, hotspot_order)

