from math import atan2, sin, cos, acos

import numpy as np
import shapely.geometry as sg
from shapely.ops import nearest_points
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from trash_utils.cc_utils import calc_mowing_lawn, search_for_trash
from trash_utils.fourD_utils import cost_to_waypoint_v1, follow_path_waypoints

from hs4dpp_main_utils import calc_heuristic_denom, calc_all_headings

import pdb

'''
All the path planning methods for hotspot_sampling_4dpp.py

'''

def calc_ball_cost(current_pos, end_pos, to_be_visited_dict, visited_dict, ball_cart_pos,
                   parent_eq_cost, time_at_node_sec, parent_energy_cost, parent_path, 
                   uuv, env, desired_speed, goal_dist, uuv_end_threshold, heuristic_denom,
                    *args):
    '''
    Calculates the appropriate eq_cost, energy, and time costs for each position
    in the cartesian ball.
    Updates the to_be_visited_dict values with this new information

    Inputs:
        current_pos (np.ndarray)   : 
        goal_pos (np.ndarray)      : 
        to_be_visited_dict (Dict)  : 
        visited_dict (Dict)        : 
        ball_cart_pos (np.ndarray) : 
        parent_eq_cost (float)     :
        time_at_node_sec (float)   :
        parent_energy_cost (float) :
        parent_path (np.ndarray)   :

    *args in order:
        vis (bool)     : 
        p (matplotlib) : 

    Returns:
        to_be_visited_dict (Dict) : 
        visited_dict (Dict)       : 

    '''
    np_args = np.array(args)
    if np_args[0]:
        ax1 = np_args[1].gca(projection='3d')

    ##Check if the end position is actually within the ball of the uuv
    ##If yes, then calculate the distance directly to it and don't calculate the ball
    ##TODO: Do we need this section?
    if goal_dist > uuv_end_threshold and (np.linalg.norm([end_pos - current_pos]) < goal_dist):
        energy_cost, time_traveled_sec, eq_cost, empty = follow_path_waypoints(
            np.array([end_pos]), current_pos, uuv, env, desired_speed, *[False])    

        self_and_parent_eq_cost = parent_eq_cost + eq_cost
        heuristic_cost = 0
        self_and_parent_time_cost_sec = time_at_node_sec + time_traveled_sec
        self_and_parent_energy_cost = parent_energy_cost + energy_cost
        new_parent_path = np.vstack((parent_path, current_pos))

        ##Append this cost to the unvisited list
        to_be_visited_dict[tuple(end_pos)] = {
                                'eq_cost_all' : self_and_parent_eq_cost, 
                                'heuristic_cost': heuristic_cost, 
                                'total_eq_cost': heuristic_cost
                                                + self_and_parent_eq_cost,  

                                'time_sec_at_this_node' : self_and_parent_time_cost_sec,

                                'total_energy_cost': self_and_parent_energy_cost,

                                'parent_path': new_parent_path}
        print ("AHA. I should end now")
        # pdb.set_trace()
        return to_be_visited_dict, visited_dict


    ##For each heading
    ##Calculate the point 10km into the distance
    ##Calculate the cost of getting to that point + parent cost = total cost
    ##Calculate the heuristic of getting to that point
    ##Save the point to the to_be_visted_list
    for angle_idx in range(ball_cart_pos.shape[0]):

        ##Calculate the desired point 10m into the distance
        ##Or if the goal is less than the designated self.goal_dist, make that the dist
        desired_cart_pos = ball_cart_pos[angle_idx]
        goal_x, goal_y, goal_z = desired_cart_pos*goal_dist + current_pos

        if abs(goal_x) > env.width/2:
            goal_x = env.width/2 * np.sign(goal_x)
        if abs(goal_y) > env.height/2:
            goal_y = env.height/2 * np.sign(goal_y)
        if goal_z > 0:
            goal_z = 0
        if goal_z < -env.max_depth:
            goal_z = -env.max_depth
        goal_pos = np.array([goal_x, goal_y, goal_z])

        ##Check that the goal_x dist is within range to the goal or make it shorter
        ##If the distance to the goal_pos is greater than the distance to the end_pos, 
        ##make goal_pos the end_pos
        # print ("dist to end  : ", np.linalg.norm([end_pos - current_pos]))
        # print ("dist to goal : ", np.linalg.norm([goal_pos - current_pos]))
        if np.linalg.norm([end_pos - current_pos]) < np.linalg.norm([goal_pos - current_pos]):
            # pdb.set_trace()
            goal_pos = end_pos

        ##If this point is within some distance to a previously calculated point,
        ## then move onto next step
        ##Update all of the stats
        not_valid = False
        # print ("not_valid: ", not_valid)
        # print ("goal_dist -5 :", self.goal_dist-5)
        # pdb.set_trace()
        # print ("Switching to visited_dict")
        for k in visited_dict.keys():
            # print ("k in visited_dict: ", k)
            check_dist = abs(np.linalg.norm([goal_pos - np.array(k)]))
            # print ("check_dist: ", check_dist)
            # print ("if state  : ", check_dist < self.goal_dist-5)
            if check_dist < goal_dist-5:
                not_valid = True
                ##Update this dict if this heuristic is smaller than what it has currently
                ##Add it back to the to_be_visited_dict
                break          

        # print ("switching to to_be_visited_dict")
        for k in to_be_visited_dict.keys():
            # print ("k: ", k)
            check_dist = abs(np.linalg.norm([goal_pos - np.array(k)]))
            # print ("check_dist: ", check_dist)
            # print ("if state  : ", check_dist < self.goal_dist-5)
            if check_dist < goal_dist-5:
                not_valid = True
                ##Update this
                break

        # print ("finished with checking dict. not_valid value: ", not_valid)
        if not_valid:
            # print ("not_valid is True so breaking into next angle_idx")
            continue

        # print ("not valid was False, so continue with the rest of the for loop")

        ##Calculate the cost of getting to that goal
        energy_cost, time_traveled_sec, eq_cost, empty = follow_path_waypoints(
            np.array([goal_pos]), current_pos, uuv, env, desired_speed, *[False])

        self_and_parent_eq_cost = parent_eq_cost + eq_cost
        heuristic_cost = ((np.linalg.norm([goal_pos-end_pos]))/heuristic_denom)
        heuristic_cost *= uuv.E_appr
        self_and_parent_time_cost_sec = time_at_node_sec + time_traveled_sec
        self_and_parent_energy_cost = parent_energy_cost + energy_cost
        new_parent_path = np.vstack((parent_path, current_pos))

        ##Append this cost to the unvisited list
        to_be_visited_dict[tuple(goal_pos)] = {
                                'eq_cost_all' : self_and_parent_eq_cost, 
                                'heuristic_cost': heuristic_cost, 
                                'total_eq_cost': heuristic_cost
                                                + self_and_parent_eq_cost,  
                                'time_sec_at_this_node' : self_and_parent_time_cost_sec,
                                'total_energy_cost': self_and_parent_energy_cost,
                                'parent_path': new_parent_path}

        ##plot where the resulting point is and the cost of how much it takes to get to that point
        if np_args[0]:
            ax1.plot([goal_x], [goal_y], [goal_z], 'bo')
            # ax1.plot([self.uuv.pos[0], goal_x],
            #          [self.uuv.pos[1], goal_y],
            #          [self.uuv.pos[2], goal_z], 'b--')
            # ax1.text(goal_x, goal_y, goal_z, str(round(heuristic_cost)))
            # ax1.text(goal_x, goal_y, goal_z, str(self_and_parent_eq_cost))
            # ax1.text(goal_x, goal_y, goal_z, str(round(heuristic_cost))+"\n"+str(round(self_and_parent_eq_cost)))

    return to_be_visited_dict, visited_dict


def find_optimal_path_nrmpc(time_start_sec, start_pos, end_pos, ball_cart_pos,
                            uuv, env, heuristic_denom, desired_speed, goal_dist,
                            uuv_end_threshold, max_num_epochs, *args):
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

    '''

    np_args = np.array(args)
    vis_args = [False]

    ##Do sorted dictionary to hold the cost, heuristic_cost, and total_cost values
    visited_dict = {}  ##add in the new values once they've been calculated
    to_be_visited_dict = {} ##This will just be the sorted dict keys list

    ##init starting node and its cost
    ##heuristic cost is taken from the paper
    print ("dist: ", np.linalg.norm([end_pos - start_pos]))
    heuristic_cost = ( (np.linalg.norm([start_pos - end_pos]))/heuristic_denom )
    heuristic_cost *= uuv.E_appr

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

                                    'total_energy_cost': 0,

                                    'parent_path': []}

    last_current_pos = np.copy(start_pos)
    current_pos = np.copy(start_pos)
    parent_eq_cost = 0.0
    time_at_node_sec = float(time_start_sec)
    parent_energy_cost = 0.0
    parent_path = np.array(np.copy(start_pos))
    
    final_eq_cost = np.inf
    final_time_sec_cost = 0.0
    final_energy_cost = np.inf
    found_path = []

    ##Visualization
    if np_args[0]:
        fig = plt.figure()
        ax1 = fig.gca(projection='3d')
        ax1.plot([current_pos[0]], [current_pos[1]], [current_pos[2]], 'go')
        ax1.text(current_pos[0], current_pos[1], current_pos[2], 'uuv.pos')
        ax1.plot([end_pos[0]], [end_pos[1]], [end_pos[2]], 'ro')
        ax1.text(end_pos[0], end_pos[1], end_pos[2], 'end_pt')
        vis_args = [True, fig]


    ##While still haven't reached the end point,
    ##calculate all of the headings and their energy costs
    epoch = 0
    while abs(np.linalg.norm([current_pos - end_pos])) > uuv_end_threshold \
          and epoch < max_num_epochs:
        # fig = plt.figure()
        # ax1 = fig.gca(projection='3d')
        # ax1.plot([current_pos[0]], [current_pos[1]], [current_pos[2]], 'go')
        # ax1.text(current_pos[0], current_pos[1], current_pos[2], 'uuv.pos')
        # ax1.plot([end_pos[0]], [end_pos[1]], [end_pos[2]], 'ro')
        # ax1.text(end_pos[0], end_pos[1], end_pos[2], 'end_pt')

        ##For each heading
        ##Calculate the point 10km into the distance
        ##Calculate the cost of getting to that point + parent cost = total cost
        ##Calculate the heuristic of getting to that point
        ##Save the point to the to_be_visted_list
        to_be_visted_dict, visited_dict = calc_ball_cost(current_pos,
                                                         end_pos, 
                                                         to_be_visited_dict, 
                                                         visited_dict, 
                                                         ball_cart_pos, 
                                                         parent_eq_cost, 
                                                         time_at_node_sec, 
                                                         parent_energy_cost,
                                                         parent_path,
                                                         uuv, env, desired_speed, 
                                                         goal_dist, 
                                                         uuv_end_threshold, 
                                                         heuristic_denom,
                                                         *vis_args)


        ##After we have calculated all these costs for all the headings, 
        ## figure out which node to calculate from next.
        ##Sort the unvisited list by cost
        current_pos = np.copy(last_current_pos)
        visited_dict[tuple(current_pos)] = to_be_visited_dict[tuple(current_pos)]
        del to_be_visited_dict[tuple(current_pos)]
        sorted_cost_dict = sorted(to_be_visited_dict.items(), key=lambda x: (x[1]['total_eq_cost']))

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

        if np_args[0]:
            ax1.plot([lowest_cost_key[0]], [lowest_cost_key[1]], [lowest_cost_key[2]], 'go')
            # ax1.text(lowest_cost_key[0], lowest_cost_key[1], lowest_cost_key[2], 'next node')

        ##Update the needed variables
        current_pos       = np.array(lowest_cost_key)
        parent_eq_cost     = lowest_cost_items['total_eq_cost']
        time_at_node_sec   = lowest_cost_items['time_sec_at_this_node']
        parent_energy_cost = lowest_cost_items['total_energy_cost']
        parent_path        = lowest_cost_items['parent_path']
        last_current_pos = np.copy(current_pos)

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
        print ("Num epochs: ", epoch, max_num_epochs)
        if abs(np.linalg.norm([current_pos - end_pos])) <= uuv_end_threshold \
              and epoch < max_num_epochs:
            print ("WE ARE WITHIN REACH TO THE END POINT!")

            final_eq_cost = parent_eq_cost
            final_time_sec_cost = time_at_node_sec
            final_energy_cost = parent_energy_cost
            found_path = parent_path
            found_path = np.vstack((parent_path, end_pos))

            ##TODO: double check all this info is correct above
            print ("final_eq_cost: ", final_eq_cost)
            print ("final_time_sec_cost: ", final_time_sec_cost)
            print ("final_energy_cost: ", final_energy_cost)
            print ("found_path: ", found_path)
            print ("start_pos: ", start_pos)
            print ("end_pos: ", end_pos)

            if np_args[0]:
                ax1.plot(found_path[:,0], found_path[:,1], found_path[:,2], 'k--')
                ax1.set_xlabel("x-axis")
                ax1.set_ylabel("y-axis")
                ax1.set_zlabel("depth")
                ax1.set_xlim(min(end_pos[0], start_pos[0]), max(end_pos[0], start_pos[0]))
                ax1.set_ylim(min(end_pos[1], start_pos[1]), max(end_pos[1], start_pos[1]))
                ax1.set_zlim(-env.max_depth, 3)
                plt.show()

            ##TODO: Save all info to a file to be compared later against the nominal solution


            return final_eq_cost, final_time_sec_cost, final_energy_cost, found_path

    if np_args[0]:
        ax1.set_xlabel("x-axis")
        ax1.set_ylabel("y-axis")
        ax1.set_zlabel("depth")
        ax1.set_xlim(min(end_pos[0], start_pos[0]), max(end_pos[0], start_pos[0]))
        ax1.set_ylim(min(end_pos[1], start_pos[1]), max(end_pos[1], start_pos[1]))
        ax1.set_zlim(-env.max_depth, 3)
        plt.show()

    print ("We did not reach the goal")
    return final_eq_cost, final_time_sec_cost, final_energy_cost, found_path