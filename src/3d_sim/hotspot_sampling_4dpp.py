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

import pdb

'''
Deploy multiple trash hotspots in an environment
Calculate a nominal path in 3D that would go from spot to spot
Do complete coverage in those spots

This is the baseline for the experiment

'''

class Nom_Simulation():

    def __init__(self):
        self.Env = Env() ##This contains (ufunc, vfunc, width, height, u_boost, v_boost)
        self.num_hotspots = 6

        self.uuv = UUV()
        
        #For 4D path planning
        self.desired_speed = 2.5722     ##meters/second (5 knots)
        self.goal_dist = 20
        # self.max_num_epochs = 15000
        self.max_num_epochs = 1000
        self.E_appr = 15 ##Approximate, minimum energy per sampling time consumed by the vehicle at each time step

        self.base_h_angle = 45


    def calc_heuristic_denom(self):
        '''
        Find the max current and calculate the heuristic denominator
        max || vc + vr ||

        TODO: CHECK THIS IS CORRECT

        '''

        d = grid_data(self.Env.dfunc, self.Env.xbound, self.Env.ybound, 50, [], [])

        ##Find the max current over a 4 day period (96 hrs total)
        min_u = 999999
        min_v = 999999
        max_u = -999999
        max_v = -999999
        for hrs in range(0, 96):
            u = grid_data(self.Env.ufunc, self.Env.xbound, self.Env.ybound, 50, d,[hrs])
            u *= self.Env.u_boost * np.random.normal(1, 0.5)
            v = grid_data(self.Env.vfunc, self.Env.xbound, self.Env.ybound, 50, d, [hrs])
            v *= self.Env.v_boost * np.random.normal(1, 0.5)
            min_u = min(min_u, np.min(u))
            min_v = min(min_v, np.min(v))
            max_u = max(max_u, np.max(u))
            max_v = max(max_v, np.max(v))

        ##Calculate the desired heading and position
        ##TODO: do I need to update the goal heading numbers here?
        ##The goal heading angles here are chosen arbitrarily
        goal_phi = np.deg2rad(45)
        goal_theta = np.deg2rad(45)
        mid_goal_x = self.desired_speed * sin(goal_phi) * cos(goal_theta)
        mid_goal_y = self.desired_speed * sin(goal_phi) * sin(goal_theta)
        mid_goal_z = self.desired_speed * cos(goal_phi)

        ##Calculate the needed UUV offset and at what angle
        desired_x_min = mid_goal_x - min_u
        desired_y_min = mid_goal_y - min_v
        desired_x_max = mid_goal_x - max_u
        desired_y_max = mid_goal_y - max_v
        desired_z = mid_goal_z
        uuv_vector_min = np.linalg.norm([desired_x_min, desired_y_min, desired_z])
        uuv_vector_max = np.linalg.norm([desired_x_max, desired_y_max, desired_z])

        denom = uuv_vector_min + np.linalg.norm([min_u, min_v])
        # print("max total: ", uuv_vector_max + np.linalg.norm([max_u, max_v]))
        # print ("denom: ", denom)
        # denom = uuv_vector_max + np.linalg.norm([max_u, max_v])

        # pdb.set_trace()
        return denom


    def calc_all_headings(self):
        '''
        Find the possible positions around the goal.
        For each heading, calculate where the AUV would end up.

        Returns:
            all_cart_pos (np.ndarray)        : (x,y,z) pos of where UUV should explore
                                                around its current pos
            all_sphere_headings (np.ndarray) : (phi, theta) headings of the cart pos


        '''

        # base_h_angle = 45
        num_headings = 360/self.base_h_angle
        #num_phi_angles = (180/self.base_h_angle) - 1
        
        ##Calculate the headings in spherical coordinate space
        theta = np.hstack((np.arange(0, 360, self.base_h_angle), 
                           np.arange(0, 360, self.base_h_angle),
                           np.arange(0, 360, self.base_h_angle))).tolist()
        theta.extend([0.0, 0.0])
        theta = np.deg2rad(theta)

        ##There is a smarter way to do this in an automated fashion:
        ##Create a num_phi_angles x num_headings np.ones matrix
        ##Multiply each row by np.arange(0, 180, base_h_angle)
        ##Flatten and tolist()
        phi = np.hstack((np.ones((1,num_headings))*45, 
                         np.ones((1,num_headings))*90,
                         np.ones((1,num_headings))*135))[0].tolist()
        phi.extend([0, 180])
        phi = np.deg2rad(phi)

        ##Convert spherical coordinates into cartesian coordinates
        ##Use cartesian coordinates to figure out where to estimate where to plan next
        cart_x = np.sin(phi)*np.cos(theta)
        cart_y = np.sin(phi)*np.sin(theta)
        cart_z = np.cos(phi)

        all_cart_pos = np.array(zip(cart_x, cart_y, cart_z))
        all_sphere_headings = np.array(zip(phi, theta))

        return all_cart_pos, all_sphere_headings


    def calc_ball_cost(self, current_pos, to_be_visited_dict, visited_dict,
                        ball_cart_pos, ball_sphere_headings):
        '''
        Calculates the appropriate eq_cost, energy, and time costs for each position
        in the cartesian ball.
        Updates the to_be_visited_dict values with this new information

        '''
        pass



    def find_optimal_path_nrmpc(self, time_start_sec, start_pos, end_pos, heuristic_denom, ball_cart_pos):
        '''
        Following the paper:
        Predictive motion planning for AUVs subject to strong time-varying currents and
         forecasting uncertainties (Huynh, Van T., Dunbabin, Matthew, Smith, Ryan N.) 
         (2015)

        They use a Nonlinear Robust Model Predictive Control (NRMPC) algorithm to define 
        their system's state and control params. Then use A* to explore the space and 
        control params until they get to their final destination.

        Input:
            time_start = time in seconds
            start_pos = (x,y,depth)
            end_pos = (x,y,depth)
            heuristic_denom (float) = calculated value for heuristic denominator

        Returns:
            middle_pts = all the waypoints (x,y,depth) to get to the end_pos
            middle_time = all the times it would take to get to end_pos
            ending_control = ending speed in (u,v)

        TODO: double check this is correct
        TODO: documentation formatting

        '''
        ##Do sorted dictionary to hold the cost, heuristic_cost, and total_cost values
        visited_dict = {}  ##add in the new values once they've been calculated
        to_be_visited_dict = {} ##This will just be the sorted dict keys list

        ##init starting node and its cost
        ##heuristic cost is taken from the paper
        heuristic_cost = ( abs(np.linalg.norm([end_pos - start_pos]))/heuristic_denom )
        heuristic_cost *= self.E_appr

        '''
        Equation cost: Used to determine which node gets selected next
        eq_cost_all = total cost from paper from all parent nodes to get to this location
        heuristic_cost = cost from this location to the end-goal
        total_eq_cost = eq_cost_all + heuristic_cost

        Time:
        time_sec_at_this_node = time in seconds when the uuv starts from this node
                                Should be time_sec_at_this_node (from parent node)
                                    + time to travel to this location in sec

        Energy:
        total_energy_cost = total amount of energy it takes to get to this node

        Path:
        parent_path = array of locations to get to this node

        '''
        to_be_visited_dict[tuple(start_pos)] = {
                                        'eq_cost_all' : 0, 
                                        'heuristic_cost': heuristic_cost, 
                                        'total_eq_cost': heuristic_cost,  

                                        'time_sec_at_this_node' : time_start_sec,

                                        # 'parent_energy_cost': 0,
                                        'total_energy_cost': 0,

                                        # 'parent_pos': tuple([np.inf, np.inf, np.inf]),
                                        'parent_path': []}


        ##Calculate all (x,y,z) pos and spherical headings to travel to around the uuv
        ##TODO: Can this be fed in so it doesn't have to calculate this every single time
        # ball_cart_pos, ball_sphere_headings = self.calc_all_headings()

        current_pos = np.copy(start_pos)
        parent_eq_cost = 0.0
        time_at_node_sec = float(time_start_sec)
        parent_energy_cost = 0.0
        parent_path = np.empty((0,3))
        
        # found_goal = False ##do we need this?
        final_eq_cost = np.inf
        final_time_sec_cost = 0.0
        final_energy_cost = np.inf
        found_path = []

        ##Visualization
        fig = plt.figure()
        ax1 = fig.gca(projection='3d')
        ax1.plot([current_pos[0]], [current_pos[1]], [current_pos[2]], 'go')
        ax1.text(current_pos[0], current_pos[1], current_pos[2], 'uuv.pos')
        ax1.plot([end_pos[0]], [end_pos[1]], [end_pos[2]], 'ro')
        ax1.text(end_pos[0], end_pos[1], end_pos[2], 'end_pt')


        ##While still haven't reached the end point,
        ##calculate all of the headings and their energy costs
        threshold1 = 10 ##meters
        epoch = 0
        while abs(np.linalg.norm([current_pos - end_pos])) > threshold1 \
              and epoch < self.max_num_epochs:
            # fig = plt.figure()
            # ax1 = fig.gca(projection='3d')
            # ax1.plot([current_pos[0]], [current_pos[1]], [current_pos[2]], 'go')
            # ax1.text(current_pos[0], current_pos[1], current_pos[2], 'uuv.pos')
            # ax1.plot([end_pos[0]], [end_pos[1]], [end_pos[2]], 'ro')
            # ax1.text(end_pos[0], end_pos[1], end_pos[2], 'end_pt')

            # self.calc_ball_cost
            ##For each heading
            ##Calculate the point 10km into the distance
            ##Calculate the cost of getting to that point + parent cost = total cost
            ##Calculate the heuristic of getting to that point
            ##Save the point to the to_be_visted_list
            for angle_idx in range(ball_cart_pos.shape[0]):
                # print ("ANGLE_IDX: ", angle_idx)

                ##Calculate the desired point 10m into the distance
                desired_cart_pos = ball_cart_pos[angle_idx]

                ##New method here?
                goal_x, goal_y, goal_z = desired_cart_pos*self.goal_dist + current_pos

                if abs(goal_x) > self.Env.width/2:
                    goal_x = self.Env.width/2 * np.sign(goal_x)
                if abs(goal_y) > self.Env.height/2:
                    goal_y = self.Env.height/2 * np.sign(goal_y)

                # print ("goal_z value. Check if neg: ", goal_z)
                # pdb.set_trace()
                # print ("UUV pos before z change: ", current_pos) 
                # print ("goal_z before: ", goal_z)
                if goal_z > 0:
                    goal_z = 0
                if goal_z < -self.Env.max_depth: ##TODO: check this makes sense
                    goal_z = -self.Env.max_depth
                goal_pos = np.array([goal_x, goal_y, goal_z])

                # print ("goal_pos: ", goal_pos)
                # pdb.set_trace()

                ##If this point is within some distance to a previously calculated point,
                ## then move onto next step
                ##I should be updating the values with better ones if I find it
                ##Update all of the stats
                not_valid = False
                # print ("not_valid: ", not_valid)
                # print ("goal_dist -5 :", self.goal_dist-5)
                # pdb.set_trace()
                for k in to_be_visited_dict.keys():
                    # print ("k: ", k)
                    check_dist = abs(np.linalg.norm([goal_pos - np.array(k)]))
                    # print ("check_dist: ", check_dist)
                    # print ("if state  : ", check_dist < self.goal_dist-5)
                    ##TODO: What does this check for below?
                    # check_dist_to_parent = abs(np.linalg.norm(v['parent_pos']-current_pos))

                    # if check_dist < self.goal_dist-5 and check_dist_to_parent>5:
                    if check_dist < self.goal_dist-5:
                        not_valid = True
                        break

                # print ("Switching to visited_dict")
                for k in visited_dict.keys():
                    # print ("k in visited_dict: ", k)
                    check_dist = abs(np.linalg.norm([goal_pos - np.array(k)]))
                    # print ("check_dist: ", check_dist)
                    # print ("if state  : ", check_dist < self.goal_dist-5)
                    if check_dist < self.goal_dist-5:
                        not_valid = True
                        break                

                # print ("finished with checking dict. not_valid value: ", not_valid)
                if not_valid:
                    # print ("not_valid is True so breaking into next angle_idx")
                    continue

                # print ("not valid was False, so continue with the rest of the for loop")

                ##Calculate the cost of getting to that goal
                ##(cost is determined by formula from paper)
                energy_cost, time_traveled_sec, eq_cost, empty = follow_path_waypoints(
                    np.array([goal_pos]), current_pos, self.uuv, self.Env, self.desired_speed, *[False])

                # print ("eq_cost: ", eq_cost)
                # print ("time_traveled_sec: ", time_traveled_sec)
                # print ("energy_cost: ", energy_cost)
                # pdb.set_trace()
                ##returns costs here?

                self_and_parent_eq_cost = parent_eq_cost + eq_cost
                heuristic_cost = (abs(np.linalg.norm([end_pos-goal_pos]))/heuristic_denom)
                # pdb.set_trace()
                heuristic_cost *= self.E_appr
                self_and_parent_time_cost_sec = time_at_node_sec + time_traveled_sec
                self_and_parent_energy_cost = parent_energy_cost + energy_cost
                new_parent_path = np.vstack((parent_path, current_pos))

                # if tuple(goal_pos) in to_be_visited_dict:
                #   print ("this key already exists in dict")
                #   print ("goal_pos: ", goal_pos)
                #   print ("current_pos: ", current_pos)
                #   print (to_be_visited_dict[tuple(goal_pos)])
                # else:
                #   print ("This key is not in the dict")

                ##Append this cost to the unvisited list
                to_be_visited_dict[tuple(goal_pos)] = {
                                        'eq_cost_all' : self_and_parent_eq_cost, 
                                        'heuristic_cost': heuristic_cost, 
                                        'total_eq_cost': heuristic_cost
                                                        + self_and_parent_eq_cost,  

                                        'time_sec_at_this_node' : self_and_parent_time_cost_sec,

                                        # 'parent_energy_cost': 0,
                                        'total_energy_cost': self_and_parent_energy_cost,

                                        # 'parent_pos': tuple([np.inf, np.inf, np.inf]),
                                        'parent_path': new_parent_path}

                ##plot where the resulting point is and the cost of how much it takes to get to that point
                ax1.plot([goal_x], [goal_y], [goal_z], 'bo')
                # ax1.plot([self.uuv.pos[0], goal_x],
                #          [self.uuv.pos[1], goal_y],
                #          [self.uuv.pos[2], goal_z], 'b--')
                # ax1.text(goal_x, goal_y, goal_z, str(heuristic_cost))
                # ax1.text(goal_x, goal_y, goal_z, str(self_and_parent_eq_cost))
                # ax1.text(goal_x, goal_y, goal_z, str(heuristic_cost+self_and_parent_eq_cost))

            ##end method

            ##After we have calculated all these costs for all the headings, 
            ## figure out which node to calculate from next.
            ##Sort the unvisited list by cost
            visited_dict[tuple(current_pos)] = to_be_visited_dict[tuple(current_pos)]
            del to_be_visited_dict[tuple(current_pos)]
            sorted_cost_dict = sorted(to_be_visited_dict.items(), key=lambda x: (x[1]['total_eq_cost']))
            # sorted_cost_dict = sorted(to_be_visited_dict.items(), key=lambda x: (x[1]['total_energy_cost']))

            # for m in range(len(sorted_cost_dict)):
            #     print (sorted_cost_dict[m][0])
            #     print ("tot eng: ", sorted_cost_dict[m][1]['total_energy_cost'])
            #     print ("time s : ", sorted_cost_dict[m][1]['time_sec_at_this_node'])
            #     print ("eq cost: ", sorted_cost_dict[m][1]['eq_cost_all'])
            #     print ("heurist: ", sorted_cost_dict[m][1]['heuristic_cost'])
            #     print ("dist   : ", abs(np.linalg.norm([np.array(sorted_cost_dict[m][0]) - end_pos])))
            #     print ("tot eq : ", sorted_cost_dict[m][1]['total_eq_cost'])
            # pdb.set_trace()

            ##Pick the next node to visit
            lowest_cost_key = sorted_cost_dict[0][0]
            lowest_cost_items = sorted_cost_dict[0][1]
            # print ("next node to be visited: ", lowest_cost_key, lowest_cost_items)
            # print ("dist to goal: ", abs(np.linalg.norm([end_pos - lowest_cost_key])))
            # print ("lowest eq cost: ", lowest_cost_items['eq_cost_all'])
            # print ("lowest heuristic_cost: ", lowest_cost_items['heuristic_cost'])
            # print ("lowest total eq cost: ", lowest_cost_items['total_eq_cost'])
            # print ()

            ax1.plot([lowest_cost_key[0]], [lowest_cost_key[1]], [lowest_cost_key[2]], 'go')
            # ax1.text(lowest_cost_key[0], lowest_cost_key[1], lowest_cost_key[2], 'next node')

            ##Update the needed variables
            current_pos       = np.array(lowest_cost_key)
            parent_eq_cost     = lowest_cost_items['total_eq_cost']
            time_at_node_sec   = lowest_cost_items['time_sec_at_this_node']
            parent_energy_cost = lowest_cost_items['total_energy_cost']
            parent_path        = lowest_cost_items['parent_path']

            # print ("next chosen node: ")
            # print ("current pos: ", current_pos)
            # print ("parent_eq_cost: ", parent_eq_cost)
            # print ("time_at_node_sec: ", time_at_node_sec)
            # print ("parent_energy_cost: ", parent_energy_cost)
            # pdb.set_trace()

            epoch += 1
            print ("epoch: ", epoch)


            ##Check if we have finished or not
            print ("CHECK IF FIN")
            print ("Remaining dist to goal: ", abs(np.linalg.norm([current_pos - end_pos])))
            print ("Num epochs: ", epoch, self.max_num_epochs)
            if abs(np.linalg.norm([current_pos - end_pos])) <= threshold1 \
                  and epoch < self.max_num_epochs:
                print ("WE ARE WITHIN REACH TO THE END POINT!")

                final_eq_cost = parent_eq_cost
                final_time_sec_cost = time_at_node_sec
                final_energy_cost = parent_energy_cost
                found_path = parent_path

                ##TODO: double check all this info is correct above
                print ("final_eq_cost: ", final_eq_cost)
                print ("final_time_sec_cost: ", final_time_sec_cost)
                print ("final_energy_cost: ", final_energy_cost)
                print ("final_path: ", final_path)
                print ("start_pos: ", start_pos)
                print ("end_pos: ", end_pos)


                # ##backtrack through the nodes as a result
                # print ("backtracking through the path")
                # found_path = (lowest_cost_items['parent_nodes'])
                # found_path = np.vstack((found_path, current_pos, end_pos))
                # found_cost = lowest_cost_items['parent_control_cost']
                # final_cost = found_cost + final_cost
                # time_sec_cost = time_at_node_sec + last_leg_time_traveled

                ax1.plot(found_path[:,0], found_path[:,1], found_path[:,2], 'k--')

                pdb.set_trace()
                return final_eq_cost, final_time_sec_cost, final_energy_cost, found_path


        ax1.plot(parent_path[:,0], parent_path[:,1], parent_path[:,2], 'g')
        ax1.set_xlabel("x-axis")
        ax1.set_ylabel("y-axis")
        ax1.set_zlabel("depth")
        ax1.set_xlim(min(end_pos[0], start_pos[0]), max(end_pos[0], start_pos[0]))
        ax1.set_ylim(min(end_pos[1], start_pos[1]), max(end_pos[1], start_pos[1]))
        ax1.set_zlim(-self.Env.max_depth, 3)
        plt.show()

        print ("We did not reach the goal")
        return final_eq_cost, final_time_sec_cost, final_energy_cost, found_path


    def calculate_paths(self, hotspot_dict, wpt_spacing, heuristic_denom, ball_cart_pos):
        '''
        Calculates a path between each hotspot by
        - Calculating closest points between two hotspots
        - Calling find_optimal_path_nrmpc to find the path between the two points

        Path to cover each hotspot is a mowing the lawn pattern

        Inputs:
            hotspot_dict (Dict)    : key = hotspot id number (1,2,..., num_hotspots)
                                     values = latest pos of all the trash in the hotspot
            wpt_spacing (int)      : distance (meters) between the nominal waypoints
            heuristic_denom (float): 

        Returns:
            cc_paths (np.ndarray)  : waypoints to do complete coverage of the hotspot
            all_paths (np.ndarray) : Nominal waypoints to travel from hotspot to hotspot
                                     To access the path from hotspot 1 to 4 for example:
                                     all_paths[1][4]

        TODO: handle return values here
        TODO: handle new inputs to here
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
                    hotspot_dict = update_trash_pos(hotspot_dict, diff_hrs, self.Env)

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
                # cc_depths = self.Env.dfunc(cc_wpts)
                # cc_paths.append(np.hstack((cc_wpts, cc_depths.reshape(-1,1))))

                hotspot_paths = []
                for b_idx in range(self.num_hotspots):
                    if b_idx == a_idx:
                        hotspot_paths.append([])
                        continue

                    convexhull_b = sg.MultiPoint(hotspot_dict[b_idx][-1][:, 0:2]).convex_hull
                    buffer_b = convexhull_b.buffer(5)
                    bx, by = buffer_b.exterior.coords.xy

                    ##Calculate the closest points between the hotspots
                    pt1, pt2 = nearest_points(buffer_a, buffer_b)
                    pt1_depth = self.Env.dfunc([pt1.x, pt1.y])[0]
                    pt2_depth = self.Env.dfunc([pt2.x, pt2.y])[0]
                    pt1_3d = np.array([pt1.x, pt1.y, pt1_depth])
                    pt2_3d = np.array([pt2.x, pt2.y, pt2_depth])

                    ##TODO: handle time_arr thing and appropriate costs
                    astar_path, astar_cost, astart_time_sec = self.find_optimal_path_nrmpc(start_time_sec, pt1_3d, pt2_3d, heuristic_denom, ball_cart_pos)

                    hotspot_paths.append(astar_path)
                all_paths.append(hotspot_paths)

            return cc_paths, all_paths




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
                                     trash_sigma, 20, self.Env.dfunc)

        ##Step the hotspot dict some x num times into the future
        # fig = plt.figure()
        for xx in range(1000):
            hotspot_dict = update_trash_pos(hotspot_dict, 0, self.Env)
            ##Uncomment to keep map view standardized:
            # visualize_trash_step(hotspot_dict, [True, fig, self.Env.map_dim]) 

            ##Use this for unstandardized map view:
            # visualize_trash_step(hotspot_dict, [True, fig])
            # plt.pause(0.05)
        # plt.show()


        ##Calculate heuristic denominator
        heuristic_denom = self.calc_heuristic_denom()

        ##Calculate all complete coverage and inter-hotspotpaths
        ## over a 4 day period in 15 min increments
        ball_cart_pos, ball_sphere_headings = self.calc_all_headings()
        self.calculate_paths(hotspot_dict, 50, heuristic_denom, ball_cart_pos)


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
        total_energy_cost = 0
        total_time_sec = 0
        total_paper_cost = 0


        ##Calculate all complete coverage and inter-hotspot paths
        # all_cc_paths, all_hotspot_paths = self.calculate_nominal_paths(hotspot_dict, 50)

        ## Arbitrarily selected order of hotspot traversal
        ## Get the path to follow
        # hotspot_order = [0, 1, 2, 3, 4, 5]
        # nominal_path = []
        # for idx in range(len(hotspot_order)-1):
        #     nominal_path.extend([all_cc_paths[hotspot_order[idx]], 
        #                          all_hotspot_paths[hotspot_order[idx]]\
        #                                           [hotspot_order[idx+1]]])
        # nominal_path.append(all_cc_paths[5])

        # ##Execute the path
        # total_energy_cost, total_time_sec, total_paper_cost = self.follow_path_order(
        #                                                         nominal_path,
        #                                                         hotspot_dict)

        print ("FINAL COST VALUES:")
        print ("total energy  : ", total_energy_cost)
        print ("total time (s): ", total_time_sec)
        print ("total cost eq : ", total_paper_cost)

if __name__ == '__main__':
    HS = Nom_Simulation()
    HS.main()