from math import atan2, sin, cos, acos

import numpy as np
import shapely.geometry as sg
from shapely.ops import nearest_points
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from trash_utils.Env import Env
from trash_utils.UUV import UUV
from trash_utils.finder_utils import (grid_data, fix_waypoint_edges)
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
Deploy multiple trash hotspots in an environment
Calculate a path in 3D that would go from spot to spot using the following paper:

    Predictive motion planning for AUVs subject to strong time-varying currents and
     forecasting uncertainties (Huynh, Van T., Dunbabin, Matthew, Smith, Ryan N.) 
     (2015)

    They use a Nonlinear Robust Model Predictive Control (NRMPC) algorithm to define 
    their system's state and control params. Then use A* to explore the space and 
    control params until they get to their final destination.

Do complete coverage in those spots

'''

class Nom_Simulation():

    def __init__(self):
        self.env = Env() ##This contains (ufunc, vfunc, width, height, u_boost, v_boost)
        self.num_hotspots = 6

        self.uuv = UUV()
        
        #For 4D path planning
        self.desired_speed = 2.5722     ##meters/second (5 knots)
        self.goal_dist = 20
        self.uuv_end_threshold = 10
        self.max_num_epochs = 15000

        self.heuristic_denom = calc_heuristic_denom(self.env, self.desired_speed)
        self.base_h_angle = 45


    def calculate_paths(self, hotspot_dict, wpt_spacing, ball_cart_pos):
        '''
        Calculates a path between each hotspot by
        - Calculating closest points between two hotspots
        - Calling find_optimal_path_nrmpc to find the path between the two points

        Path to cover each hotspot is a mowing the lawn pattern

        Inputs:
            hotspot_dict (Dict)        : key = hotspot id number (1,2,..., num_hotspots)
                                        values = latest pos of all the trash in the hotspot
            wpt_spacing (int)          : distance (meters) between the nominal waypoints
            heuristic_denom (float)    :  
            ball_cart_pos (np.ndarray) : 

        Returns:
            cc_paths (np.ndarray)  : waypoints to do complete coverage of the hotspot
            all_paths (np.ndarray) : Nominal waypoints to travel from hotspot to hotspot
                                     To access the path from hotspot 1 to 4 for example:
                                     all_paths[1][4]

        TODO: handle return values here
        '''

        ##For each of these hotspots, 
        ##Calculate the path to each other 
        ## and the complete coverage algorithm to cover them
        ##TODO: update this with some other data structure/add the ones for keeping track of costs
        cc_paths = []
        all_paths = []

        ##Time here is (0, days x hrs/day x mins/hr x sec/min, mins x sec/min)
        time_span_sec = np.arange(0, 1*24*60*60, 15*60)
        for ts in range(len(time_span_sec)):
            start_time_sec = time_span_sec[ts]
            start_time_hrs = start_time_sec/3600

            ##Update the trash positions up to this time
            if ts > 0:
                for diff_sec in range(time_span_sec[ts-1], time_span_sec[ts]):
                    diff_hrs = diff_sec/3600
                    hotspot_dict = update_trash_pos(hotspot_dict, diff_hrs, self.env)

            ##Calculate paths from hotspot to hotspot at this time
            for a_idx in range(self.num_hotspots):
                ## Get convex hull of each hotspot plus some buffer
                convexhull_a = sg.MultiPoint(hotspot_dict[a_idx][-1][:, 0:2]).convex_hull
                buffer_a = convexhull_a.buffer(5)
                ax, ay = buffer_a.exterior.coords.xy
                [minx, miny, maxx, maxy] = buffer_a.bounds
                cc_y_lines = np.arange(miny, maxy, 5)

                # ##Calculate the CC pattern on this hotspot
                # ##TODO: Figure out what to do about cc_paths
                # cc_wpts = calc_mowing_lawn(buffer_a, cc_y_lines)
                # cc_depths = self.env.dfunc(cc_wpts)
                # cc_paths.append(np.hstack((cc_wpts, cc_depths.reshape(-1,1))))

                hotspot_paths = []
                for b_idx in range(3, self.num_hotspots):
                    if b_idx == a_idx:
                        hotspot_paths.append([])
                        continue

                    convexhull_b = sg.MultiPoint(hotspot_dict[b_idx][-1][:, 0:2]).convex_hull
                    buffer_b = convexhull_b.buffer(5)
                    bx, by = buffer_b.exterior.coords.xy

                    ##Calculate the closest points between the hotspots
                    pt1, pt2 = nearest_points(buffer_a, buffer_b)
                    pt1_depth = self.env.dfunc([pt1.x, pt1.y])[0]
                    pt2_depth = self.env.dfunc([pt2.x, pt2.y])[0]
                    pt1_3d = np.array([pt1.x, pt1.y, pt1_depth])
                    pt2_3d = np.array([pt2.x, pt2.y, pt2_depth])

                    ##TODO: handle time_arr thing and appropriate costs
                    astr_eq_cost, astr_time_cost_sec, astr_energy_cost, astr_path = find_optimal_path_nrmpc(start_time_sec, pt1_3d, pt2_3d, ball_cart_pos)

                    print ("REACHED A COST")
                    pdb.set_trace()
                    hotspot_paths.append(astr_path)
                all_paths.append(hotspot_paths)

        return cc_paths, all_paths


    def calculate_compare_paths(self, hotspot_dict, wpt_spacing, ball_cart_pos, path_order, time_start_sec):
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
            cc_wpts = fix_waypoint_edges(cc_wpts, self.env)
            ##Calculate the cost of traveling this cc_path at this time
            energy_cost, time_cost_sec, est_cost = search_for_trash(cc_wpts, 
                                                                    hotspot_dict, 
                                                                    self.uuv, 
                                                                    self.env, 
                                                                    self.desired_speed,
                                                                    time_start_sec, 
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

        return cc_paths, all_paths


    def main_find_best_path(self, hotspot_dict, ball_cart_pos):
        ##Calculate the costs of all the paths
        ##TODO: figure out what paths need to be calculated here
        self.calculate_paths(hotspot_dict, 50, ball_cart_pos)

        ##TODO: GET PATH ORDER
        ##Get the best path cost over a 1 day time period
        ##Use that as a starting node
        greedy_tsp_order = []
        greedy_tsp_path = []


        ##Find the smallest path to the next node after that time
        ##In that column for that row.
        ##Repeat for each path until all hotspots have been visited

        ##Now that we have the path order
        ##Calculate the cost of traveling this whole route
        # total_energy_cost = 0.0
        # total_time_sec = 0.0
        # total_paper_cost = 0.0

        # for idx in range(len(hotspot_order)-1):
        #     nominal_path.extend([all_cc_paths[hotspot_order[idx]], 
        #                          all_hotspot_paths[hotspot_order[idx]]\
        #                                           [hotspot_order[idx+1]]])
        # nominal_path.append(all_cc_paths[5])

        # ##Execute the path
        # self.uuv.pos = np.array([0.0, 0.0, 0.0])
        # self.uuv.heading_rad = 0.0
        # total_energy_cost, total_time_sec, total_paper_cost = self.follow_path_order(
        #                                                         nominal_path,
        #                                                         hotspot_dict)

        # print ("FINAL COST VALUES:")
        # print ("total energy  : ", total_energy_cost)
        # print ("total time (s): ", total_time_sec)
        # print ("total cost eq : ", total_paper_cost)


    def main_compare_nom(self, hotspot_dict, ball_cart_pos):
        ## Arbitrarily selected order of hotspot traversal
        ##Calculate the cost of traveling this whole route
        hotspot_order = [0, 1, 2, 3, 4, 5]
        total_energy_cost, total_time_sec, total_paper_cost, total_path = self.calculate_compare_paths(hotspot_dict, 50, ball_cart_pos, hotspot_order)

        ##Execute the path
        # self.uuv.pos = np.array([0.0, 0.0, 0.0])
        # self.uuv.heading_rad = 0.0
        # total_energy_cost, total_time_sec, total_paper_cost = self.follow_path_order(
        #                                                         nominal_path,
        #                                                         hotspot_dict)

        print ("FINAL COST VALUES:")
        print ("total energy  : ", total_energy_cost)
        print ("total time (s): ", total_time_sec)
        print ("total cost eq : ", total_paper_cost)



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

        ##Step the hotspot dict some x num times into the future
        # fig = plt.figure()
        for xx in range(1000):
            hotspot_dict = update_trash_pos(hotspot_dict, 0, self.env)
            ##Uncomment to keep map view standardized:
            # visualize_trash_step(hotspot_dict, [True, fig, self.env.map_dim]) 

            ##Use this for unstandardized map view:
            # visualize_trash_step(hotspot_dict, [True, fig])
            # plt.pause(0.05)
        # plt.show()

        ##Calculate all complete coverage and inter-hotspotpaths
        ## over a 4 day period in 15 min increments
        ball_cart_pos, ball_sphere_headings = calc_all_headings(self.base_h_angle)

        # self.main_find_best_path(hotspot_dict, ball_cart_pos)
        self.main_compare_nom(hotspot_dict, ball_cart_pos)


if __name__ == '__main__':
    HS = Nom_Simulation()
    HS.main()