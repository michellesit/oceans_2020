from math import atan2, sin, cos, acos

import numpy as np
import shapely.geometry as sg
from shapely.ops import nearest_points
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from trash_utils.cc_utils import calc_mowing_lawn, search_for_trash
from trash_utils.fourD_utils import cost_to_waypoint_v1, follow_path_waypoints



import pdb

'''
All the general class methods for hotspot_sampling_4dpp.py


'''


def calc_heuristic_denom(env, desired_speed):
    '''
    Find the max current value in the map and calculate the heuristic denominator from the paper 
    max || vc + vr ||

    [TODO: Put in the actual equation here]

    Inputs:
        env (object)          : 
        desired_speed (float) : 

    Returns:
        denom (float) : calculated heuristic value according to the paper

    '''

    d = grid_data(env.dfunc, env.xbound, env.ybound, 50, [], [])

    ##Find the max current over a 4 day period (96 hrs total)
    # min_u = 999999
    # min_v = 999999
    max_u = -999999
    max_v = -999999
    for hrs in range(0, 96):
        u = grid_data(env.ufunc, env.xbound, env.ybound, 50, d,[hrs])
        u *= env.u_boost * np.random.normal(1, 0.5)
        v = grid_data(env.vfunc, env.xbound, env.ybound, 50, d, [hrs])
        v *= env.v_boost * np.random.normal(1, 0.5)
        # min_u = min(min_u, np.min(u))
        # min_v = min(min_v, np.min(v))
        max_u = max(max_u, np.max(u))
        max_v = max(max_v, np.max(v))

    ##Calculate the desired heading and position
    ##TODO: do I need to update the goal heading numbers here?
    ##The goal heading angles here are chosen arbitrarily
    goal_phi = np.deg2rad(45)
    goal_theta = np.deg2rad(45)
    mid_goal_x = desired_speed * sin(goal_phi) * cos(goal_theta)
    mid_goal_y = desired_speed * sin(goal_phi) * sin(goal_theta)
    mid_goal_z = desired_speed * cos(goal_phi)

    ##Calculate the needed UUV offset and at what angle
    # desired_x_min = mid_goal_x - min_u
    # desired_y_min = mid_goal_y - min_v
    desired_x_max = mid_goal_x - max_u
    desired_y_max = mid_goal_y - max_v
    desired_z = mid_goal_z
    # uuv_vector_min = np.linalg.norm([desired_x_min, desired_y_min, desired_z])
    uuv_vector_max = np.linalg.norm([desired_x_max, desired_y_max, desired_z])

    # denom = uuv_vector_min + np.linalg.norm([min_u, min_v])
    # print ("min denom: ", denom)
    denom = uuv_vector_max + np.linalg.norm([max_u, max_v])
    # print ("max denom: ", denom)

    return denom


def calc_all_headings(base_h_angle):
    '''
    Find the possible positions around the goal.
    For each heading, calculate where the AUV would end up.

    Inputs:
        base_h_angle (int) : 

    Returns:
        all_cart_pos (np.ndarray)        : (x,y,z) pos of where UUV should explore
                                            around its current pos
        all_sphere_headings (np.ndarray) : (phi, theta) headings of the cart pos

    '''

    # base_h_angle = 45
    num_headings = 360/base_h_angle
    #num_phi_angles = (180/self.base_h_angle) - 1
    
    ##Calculate the headings in spherical coordinate space
    theta = np.hstack((np.arange(0, 360, base_h_angle), 
                       np.arange(0, 360, base_h_angle),
                       np.arange(0, 360, base_h_angle))).tolist()
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