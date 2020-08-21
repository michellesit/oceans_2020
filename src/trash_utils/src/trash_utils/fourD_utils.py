from math import atan2, sin, cos, acos, isnan
from copy import deepcopy

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from trash_utils.trash_lib import visualize_trash_step
import trash_utils.cc_utils

import pdb


'''
Hotspot path planning algorithms

'''

def follow_path_waypoints(all_waypoints, current_pos, uuv, env, desired_speed, time_start_sec, *args, **kwargs):
    '''
    Calculates preliminary info (pos, heading) needed to travel from the uuv to 
    each waypoint 
    
    Inputs:
        all_waypoints (np.ndarray)  : waypoint(s) to travel to
        current_pos (np.ndarray)    : (x,y,z) uuv pos to calculate from
        uuv (Obj)                   : trash_utils/UUV.py to access (pos, max_thrust)
        env (Obj)                   : trash_utils/Env.py object to access
                                      (ufunc, vfunc, width, height, max_depth)
        desired_speed (float)       : uuv speed (meters/sec) for each step
        time_start_sec (int)        : time to start searching algorithm (in sec)

    *args in order:
        args1 (bool)                : True = visualize waypoints and uuv pos
        args2 (matplotlib)          : plt.figure() or subplot to plot points in

    **kwargs items and keys:
        'trash_dict'=all_trash_pos (np.ndarray) : all the trash piece (x,y,z) pos.
                                                  dim=(-1,3)

    Returns:
        energy_cost (float)       : amount of energy uuv needs to complete this path
        time_cost_sec (int)       : amount of time in seconds to complete this path
        eq_cost (float)           : cost calculated from Huynh, Dunbabin, Smith (ICRA 2015)
                                    Equation 6 which takes energy and time into account
        detected_pos (np.ndarray) : (x,y,z) positions of detected trash. dim=(-1,3)

    '''
    np_args = np.array(args).flatten()

    total_cost = 0
    total_energy = 0
    total_time = 0
    detected_pos = []
    # print ("UUV POS BEFORE: ", current_pos, uuv.pos)
    for pt in range(all_waypoints.shape[0]):
        ##Calculate heading to get from current_pos to the next waypoint
        goal_pt = all_waypoints[pt]
        diff = goal_pt - current_pos
        if np.linalg.norm([goal_pt - current_pos]) < 0.5:
            continue
        goal_phi = acos((diff[2]/np.linalg.norm(diff)))
        goal_theta = atan2(diff[1], diff[0])

        if np_args[0]:
            fig = np_args[1]
            ax1 = fig.add_subplot(122, projection='3d')
            ax1.plot([current_pos[0]], [current_pos[1]], [current_pos[2]], 'bo')
            ax1.text(current_pos[0], current_pos[1], current_pos[2], 'start')

            ax1.plot(all_waypoints[:,0], all_waypoints[:,1], all_waypoints[:,2], 'bo')
            ax1.plot(all_waypoints[:,0], all_waypoints[:,1], all_waypoints[:,2], 'b--')
            ax1.text(all_waypoints[-1, 0], all_waypoints[-1, 1], all_waypoints[-1, 2],
                     'goal')

            plt.show()

        save_current_pos = deepcopy(current_pos)
        cost, time_sec, controls, detected_pos = cost_to_waypoint_v1(current_pos, 
                                                                     goal_pt,
                                                                     [goal_phi, goal_theta],
                                                                     time_start_sec,
                                                                     uuv, env,
                                                                     desired_speed,
                                                                     *args, **kwargs)

        # print ("INNER LOOP")
        # print ("cost       : ")

        if isnan(cost):
            print ("cost is nan: ", cost)
        if cost > 99999999999999999999999:
            print ("cost is inf: ", cost)
        en = np.sum(controls[:,0])
        if isnan(en):
            print ("energy is nan: ", en)
            # pdb.set_trace()
        if en > 999999999999999999999999:
            print ("en is inf: ", en)

        total_cost += cost
        total_energy += np.sum(controls[:,0])
        total_time += time_sec

        # print ("cost : ", cost)
        # print ("energy: ", np.sum(controls[:,0]))
        # print ("time:   ", time_sec)

    # print ("FPW pos: ", current_pos, uuv.pos)
    uuv.pos = deepcopy(current_pos)

    return total_energy, total_time, total_cost, detected_pos




def cost_to_waypoint_v1(input_start_pos, goal_pos, goal_heading, time_now, uuv, env, \
                        desired_speed, *args, **kwargs):
    '''
    Given headings and positions, iteratively calculate the cost of getting to that position
    Returns cost as timesteps and uuv_controls

    Input:
        input_start_pos (np.ndarray) : uuv position (x,y,z)
        goal_pos (np.ndarray)        : waypoint (x,y,z)
        goal_heading (np.ndarray)    : radians [phi, theta]
        time_now (int)               : time in seconds
        uuv (Obj)                    : trash_utils/UUV.py to access (pos, max_thrust)
        env (Obj)                    : trash_utils/Env.py object to access
                                       (ufunc, vfunc, width, height, max_depth)
        desired_speed (float)        : uuv speed (meters/sec) for each step

    *args in order:
        args1 (bool)                 : True = visualize waypoints and uuv pos
        args2 (matplotlib)           : plt.figure() or subplot to plot points in

    **kwargs items and keys:
        'trash_dict'=all_trash_pos (np.ndarray) : all the latest trash piece (x,y,z) pos.
                                                  dim=(-1,3)

    Returns:
        cost (float)                  : cost calculated from Huynh, Dunbabin, Smith (2015)
                                        Equation 6 which takes energy and time into account
        timesteps (int)               : time it took to travel to the goal node in seconds
        controls (np.ndarray)         : amount of energy uuv and all the control params
                                        needed to get to the goal waypoint
        all_detected_idx (np.ndarray) : (x,y,z) positions of detected trash to this waypt.
                                        dim=(-1,3)

    '''
    np_args = np.array(args).flatten()
    np.random.seed(8)
    
    all_detected_idx = np.empty((0,3))
    uuv_controls = []

    # pos = np.copy(input_start_pos)
    pos = input_start_pos
    goal_phi = goal_heading[0]
    goal_theta = goal_heading[1]

    threshold2 = 0.5  ##meters
    # threshold2 = 50
    epoch2 = 0
    # max_num_epoch2 = 10000
    max_num_epoch2 = 30000
    first_run = True
    timesteps = 0
    while (abs(np.linalg.norm([pos - goal_pos])) > threshold2 \
          and epoch2 < max_num_epoch2) \
          or (first_run == True):

        if np_args[0]:
            fig = np_args[1]
            ax1 = fig.add_subplot(122, projection='3d')
            ##Plots start and end goal pos with blue connecting line
            ax1.plot([input_start_pos[0]], [input_start_pos[1]], [input_start_pos[2]], 'bo')
            ax1.text(input_start_pos[0], input_start_pos[1], input_start_pos[2], 'start')

            ax1.plot([goal_pos[0]], [goal_pos[1]], [goal_pos[2]], 'bo')
            ax1.text(goal_pos[0], goal_pos[1], goal_pos[2], 'goal')
            ax1.plot([input_start_pos[0], goal_pos[0]], [input_start_pos[1],
                      goal_pos[1]], [input_start_pos[2], goal_pos[2]], 'b--')

        time_now_hrs = (time_now + timesteps)/3600.0
        ##Calculate ocean u,v currents
        current_u = env.ufunc([time_now_hrs, abs(pos[2]), pos[0], pos[1]])[0] \
                  * env.u_boost * np.random.normal(1, 0.5)
        current_v = env.vfunc([time_now_hrs, abs(pos[2]), pos[0], pos[1]])[0] \
                  * env.v_boost * np.random.normal(1, 0.5)
        current_sum = current_u + current_v


        ##Calculate the desired heading and position of the uuv
        mid_goal_x = desired_speed * sin(goal_phi) * cos(goal_theta)
        mid_goal_y = desired_speed * sin(goal_phi) * sin(goal_theta)
        mid_goal_z = desired_speed * cos(goal_phi)

        ##Adapt the mid_goal in respose to distance to actual point
        ##If the mid_goal is farther than what's left to the goal point, 
        ##   make mid_goal the remaining distance
        if abs(mid_goal_x) > abs(np.linalg.norm([pos[0] - goal_pos[0]])):
            mid_goal_x = goal_pos[0] - pos[0]
        if abs(mid_goal_y) > abs(np.linalg.norm([pos[1] - goal_pos[1]])):
            mid_goal_y = goal_pos[1] - pos[1]
        if abs(mid_goal_z) > abs(np.linalg.norm([pos[2] - goal_pos[2]])):
            mid_goal_z = goal_pos[2] - pos[2]

        ##If mid_goal is outside the bounds of the map, then set to the edge
        if abs(pos[0]+mid_goal_x) > env.width/2:
            mid_goal_x = 0.0
        if abs(pos[1]+mid_goal_y) > env.height/2:
            mid_goal_y = 0.0
        if abs(pos[2]+mid_goal_z) > env.max_depth:
            mid_goal_z = 0.0

        mid_goal_sum = mid_goal_x + mid_goal_y + mid_goal_z


        ##Calculate the needed UUV offset and at what angle
        desired_x = mid_goal_x - current_u
        desired_y = mid_goal_y - current_v
        desired_z = mid_goal_z
        uuv_energy = desired_x + desired_y + desired_z
        uuv_vector = np.linalg.norm([desired_x, desired_y, desired_z])
        uuv_phi = acos((desired_z/uuv_vector))
        uuv_theta = atan2(desired_y, desired_x)

        ##If the current takes care of all of the energy on this leg, 
        ##  set the UUV control to 0
        if mid_goal_sum <= current_sum:
            uuv_energy = 0.0
            # print ("uuv_energy is 0")

        ##If the needed thrust is greater than what the uuv can provide, return inf cost
        if uuv_vector > uuv.max_thrust:
            print ("uuv_vector > max thrust: ", uuv_vector)
            # pdb.set_trace()
            return np.inf, np.inf, np.empty((0,6)), all_detected_idx

        uuv_controls.append([uuv_energy, desired_x, desired_y, desired_z,
                             uuv_phi, uuv_theta])
        timesteps += 1

        ##update pos with resulting location
        pos += [mid_goal_x, mid_goal_y, mid_goal_z]
        epoch2 += 1
        first_run = False

        #Visualize uuv pos as it travels to its next waypoint
        if np_args[0]:
            ax1.plot([pos[0]], [pos[1]], [pos[2]], 'ro')

        ##If we are searching for trash, calculate uuv distance to trash pos
        if 'trash_dict' in kwargs:
            all_trash_pos = kwargs['trash_dict']
            uuv_trash_diff = np.hypot(*(pos - all_trash_pos).T)
            detected_idx = np.where(abs(uuv_trash_diff) <= uuv.trash_detection_dist)[0]
            if len(detected_idx)>0:
                detected_trash_pos = all_trash_pos[detected_idx]
                all_detected_idx = np.vstack((all_detected_idx, detected_trash_pos))

            ##Save the uuv stuff?
            ##Create dictionary of discovered trash?

    if np_args[0]:
        ax1.set_xlabel("x-axis")
        ax1.set_ylabel("y-axis")
        ax1.set_zlabel("depth")
        plt.show()

    ##If the cost is negative (meaning the uuv used no energy), then make cost = 0
    uuv_controls = np.array(uuv_controls)
    cost = (timesteps*(12.0/3600.0)) \
         + (timesteps * (np.sum((uuv_controls[:, 1:4]**3))**(0.333333))) * 0.70
    if isnan(cost):
        cost = 0.0
    if cost > 999999999999999999999999:
        # pdb.set_trace()
        cost = 0.0

    ##Filter out all repeat detections. Only return new trash detections
    ##This could be its own method in the real sim
    ##This would be where you do data association between observations to determine
    ##  if you have detected the same two objects
    if len(all_detected_idx) > 0:
        all_detected_idx = np.unique(all_detected_idx, axis=0)

    if abs(np.linalg.norm([pos - goal_pos])) > threshold2 and epoch2 >= max_num_epoch2:
        print ("WAS NOT ABLE TO FIND A SOLUTION")
        # pdb.set_trace()
        return np.inf, np.inf, np.array(uuv_controls).reshape((-1, 6)), all_detected_idx
    else:

        en = np.sum(uuv_controls[:,0])
        # if isnan(en):
            # pdb.set_trace()
        return cost, timesteps, np.array(uuv_controls).reshape((-1, 6)), all_detected_idx


def follow_path_order(nominal_path, trash_dict, uuv, env, desired_speed, time_start_sec, 
    **kwargs):
    '''
    Switches off between following paths to the cc areas and searching for trash

    Inputs:
        nominal_path (array) : Array of arrays. Each sub-array contains the waypoints 
                               to do complete coverage within the hotspot
                               or to travel from hotspot to hotspot
        trash_dict (Dict)    : key = hotspot id number (currrent 1,2,..., num_hotspots)
                               values = latest positions of all the trash in the hotspot
        vis_dash (bool)      : True = visualize global path and 
                               path for each hotspot searching or between hotspots
        uuv (Obj)            : trash_utils/UUV.py to access (pos, max_thrust)
        env (Obj)            : trash_utils/Env.py object to access
                               (ufunc, vfunc, width, height, max_depth)
        desired_speed (float): uuv speed (meters/sec) for each step

    Returns:
        total_trip_energy_cost (float) : total amount of energy uuv uses for mission
        total_trip_time_sec (int)      : total amount of time in sec to complete mission
        total_paper_cost (float)       : total cost calculated from Huynh, Dunbabin,
                                         Smith (ICRA 2015)
                                         Equation 6 which takes energy and time into 
                                         account

    '''
    vis_dash = False
    vis_args = [False]
    
    total_trip_energy_cost = 0
    total_trip_time_sec = 0
    total_paper_cost = 0
    run_start_time_sec = total_trip_time_sec + time_start_sec

    if 'uuv_path_state' in kwargs.keys():
        print ("state in kwargs")
        uuv_path_state = kwargs['uuv_path_state']
    else:
        uuv_path_state = 'searching'

    if 'vis_args' in kwargs.keys():
        print ('vis_args in kwargs')
        vis_dash = kwargs['vis_args'][0]


    for np_idx in range(len(nominal_path)):
        currently_following_path = nominal_path[np_idx]

        if vis_dash == True:
            fig = plt.figure()
            vis_args = [True, fig]

            ##Adds 2D overview of the map
            ax1 = fig.add_subplot(121)
            np_nominal_path = np.vstack(nominal_path)
            ##whole path
            ax1.plot(np_nominal_path[:,0], np_nominal_path[:,1], 'b--')
            ax1.plot(np_nominal_path[:,0], np_nominal_path[:,1], 'bo')
            ##What the uuv will tackle next
            ax1.plot(uuv.pos[0], uuv.pos[1], 'ro') ##uuv
            ax1.plot([uuv.pos[0], currently_following_path[0,0]],
                     [uuv.pos[1], currently_following_path[0,1]], 'k')
            ax1.plot(currently_following_path[:,0], currently_following_path[:,1], 'k')
            visualize_trash_step(trash_dict, [True, ax1])

        if uuv_path_state == 'following':
            print ("following")
            energy_cost, time_cost_sec, est_cost, none = follow_path_waypoints(
                                                         currently_following_path, 
                                                                    uuv.pos,
                                                                    uuv, 
                                                                    env, 
                                                                    desired_speed,
                                                                    run_start_time_sec,
                                                                    vis_args)

        if uuv_path_state == 'searching':
            print ('searching')
            energy_cost, time_cost_sec, est_cost = trash_utils.cc_utils.search_for_trash(
                                                   currently_following_path, 
                                                                    trash_dict, 
                                                                    uuv, 
                                                                    env, 
                                                                    desired_speed,
                                                                    run_start_time_sec,
                                                                    vis_args)

        print ("COST TO TRAVEL THIS LEG OF THE TRIP")
        print ("energy cost   : ", energy_cost)
        print ("time cost (s) : ", time_cost_sec)
        print ("est cost      : ", est_cost)
        ##Add up cost to travel this leg of the trip
        total_trip_energy_cost += energy_cost
        total_trip_time_sec += time_cost_sec
        total_paper_cost += est_cost
        run_start_time_sec += time_cost_sec

        if uuv_path_state == 'path_following':
            print ("switched to searching")
            uuv_path_state = 'searching'
        elif uuv_path_state == 'searching':
            print ("switched to following")
            uuv_path_state = 'path_following'

    return total_trip_energy_cost, total_trip_time_sec, total_paper_cost


def calculate_nominal_path_short(pt1, pt2, wpt_spacing, env):
    '''
    Calculates a path between each hotspot by
    - Calculating closest points between two hotspots
    - Breaking the path down into smaller pieces
    - Setting the waypoints to be those breakpoints with depth at the bottom of the map

    Path from hotspot to hotspot is the euclidean distance at the bottom of the map
    
    TODO? Smooth out the waypoints

    Inputs:
        pt1 (np.ndarray)  : (x,y,z) of starting point
        pt2 (np.ndarray)  : (x,y,z) of ending point
        wpt_spacing (int) : distance (meters) between the nominal waypoints
        env (object)      : 

    Returns:
        all_paths (np.ndarray) : Nominal waypoints to travel from hotspot to hotspot

    '''

    num_pts = abs(pt1[0] - pt2[0])//wpt_spacing
    if num_pts == 0:
        path = np.array([pt1, pt2])
    else:
        waypts = np.array(zip(np.linspace(pt1[0], pt2[0], num_pts, endpoint=True), 
                              np.linspace(pt1[1], pt2[1], num_pts, endpoint=True)))
        waypts_depth = env.dfunc(waypts)
        path = np.hstack((waypts, waypts_depth.reshape(-1,1) ))

    return path


# def create_nom_cost_matrix(wpt1, wpt2, wpt_spacing, desired_speed, trash_dict, uuv, env, vis=True):
#     '''
#     Calculates the 

#     '''
#     nom_path = calculate_nominal_path_short(wpt1, wpt2, wpt_spacing, env)

#     time_hrs = np.arange(0, 144, 0.5)
#     energy_cost_matrix = []
#     time_cost_matrix = []
#     est_cost_matrix = []
#     for t in range(len(time)):
#         nom_energy, nom_time_sec, nom_est_cost, none = follow_path_waypoints(nom_path,
#                                                                 wpt1,
#                                                                 uuv,
#                                                                 env,
#                                                                 desired_speed,
#                                                                 time_hrs[t],
#                                                                 *[False])
#         energy_cost_matrix.append(nom_energy)
#         time_cost_matrix.append(nom_time_sec)
#         est_cost_matrix.append(nom_est_cost)

#     ##visualize
#     if vis == True:
#         fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
#         fig.suptitle('Comparing path costs for wpt1{0} to wpt2{1}')
#         ax1.plot(time_hrs, energy_cost_matrix, 'o-')
#         ax1.set_ylabel('Energy Cost')
#         ax2.plot(time_hrs, time_cost_matrix, 'o-')
#         ax2.set_ylabel('Time cost (sec)')
#         ax3.plot(time_hrs, nom_est_cost, 'o-')
#         ax3.set_ylabel('Est Cost')
#         ax3.set_xlabel('time (hrs)')
#         plt.show()

#     return np.array(energy_cost_matrix), np.array(time_cost_matrix), np.array(est_cost_matrix)