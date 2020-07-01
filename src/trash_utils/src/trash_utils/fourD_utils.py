from math import atan2, sin, cos, acos

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import pdb


'''
Hotspot path planning algorithms

'''

def follow_path_waypoints(all_waypoints, uuv, env, desired_speed, *args, **kwargs):
    '''
    Calculates preliminary info (pos, heading) needed to travel from the uuv to 
    each waypoint 
    
    Inputs:
        all_waypoints (np.ndarray)  : waypoint(s) to travel to
        uuv (Obj)                   : trash_utils/UUV.py to access (pos, max_thrust)
        env (Obj)                   : trash_utils/Env.py object to access
                                      (ufunc, vfunc, width, height, max_depth)
        desired_speed (float)       : uuv speed (meters/sec) for each step

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
    for pt in range(all_waypoints.shape[0]):
        ##Calculate heading
        goal_pt = all_waypoints[pt]
        diff = goal_pt - uuv.pos
        goal_phi = acos((diff[2]/np.linalg.norm(diff)))
        goal_theta = atan2(diff[1], diff[0])
        mid_goal = np.array((sin(goal_phi)*cos(goal_theta), 
                             sin(goal_phi)*sin(goal_theta), 
                             cos(goal_phi)))*100
        mid_goal = np.copy(uuv.pos) + mid_goal

        if np_args[0]:
            fig = np_args[1]
            ax1 = fig.add_subplot(122, projection='3d')
            ax1.plot([uuv.pos[0]], [uuv.pos[1]], [uuv.pos[2]], 'bo')
            ax1.text(uuv.pos[0], uuv.pos[1], uuv.pos[2], 'start')

            ax1.plot(all_waypoints[:,0], all_waypoints[:,1], all_waypoints[:,2], 'bo')
            ax1.plot(all_waypoints[:,0], all_waypoints[:,1], all_waypoints[:,2], 'b--')
            ax1.text(all_waypoints[-1, 0], all_waypoints[-1, 1], all_waypoints[-1, 2],
                     'goal')

            plt.show()

        cost, time_sec, controls, detected_pos = cost_to_waypoint_v1(uuv.pos, 
                                                                     goal_pt,
                                                                     [goal_phi, goal_theta],
                                                                     0,
                                                                     uuv, env,
                                                                     desired_speed,
                                                                     *args, **kwargs)

        total_cost += cost
        total_energy += np.sum(controls[:,0])
        total_time += time_sec

        print ("cost : ", cost)
        print ("energy: ", np.sum(controls[:,0]))
        print ("time:   ", time_sec)

    print ("total energy: ", total_energy)
    print ("total time  : ", total_time)
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
    
    all_detected_idx = np.empty((0,3))
    uuv_controls = []

    # pos = np.copy(input_start_pos)
    pos = input_start_pos
    goal_phi = goal_heading[0]
    goal_theta = goal_heading[1]

    threshold2 = 2  ##meters
    epoch2 = 0
    max_num_epoch2 = 300
    first_run = True
    timesteps = 0
    while (abs(np.linalg.norm([pos - goal_pos])) > threshold2 and epoch2 < max_num_epoch2) \
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
        if mid_goal_x > abs(np.linalg.norm([pos[0] - goal_pos[0]])):
            mid_goal_x = np.linalg.norm([pos[0] - goal_pos[0]])
        if mid_goal_y > abs(np.linalg.norm([pos[1] - goal_pos[1]])):
            mid_goal_y = np.linalg.norm([pos[1] - goal_pos[1]])
        if mid_goal_z > abs(np.linalg.norm([pos[2] - goal_pos[2]])):
            mid_goal_z = np.linalg.norm([pos[2] - goal_pos[2]])

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

        ##If the needed thrust is greater than what the uuv can provide,
        ##  return 9999999999 cost
        if uuv_vector > uuv.max_thrust:
            print ("uuv_vector: ", uuv_vector)
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

    print ("epoch2: ", epoch2)

    if np_args[0]:
        ax1.set_xlabel("x-axis")
        ax1.set_ylabel("y-axis")
        ax1.set_zlabel("depth")
        plt.show()

    uuv_controls = np.array(uuv_controls)
    cost = (timesteps*(12.0/3600.0)) \
         + (timesteps * (np.sum((uuv_controls[:, 0]**3))**(0.333333))) * 0.70

    if len(all_detected_idx) > 0:
        all_detected_idx = np.unique(all_detected_idx, axis=0)

    if abs(np.linalg.norm([pos - goal_pos])) > threshold2 and epoch2 >= max_num_epoch2:
        print ("WAS NOT ABLE TO FIND A SOLUTION")
        return np.inf, np.inf, np.array(uuv_controls).reshape((-1, 6)), all_detected_idx
    else:
        return cost, timesteps, np.array(uuv_controls).reshape((-1, 6)), all_detected_idx