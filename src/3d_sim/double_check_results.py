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
from trash_utils.fourD_utils import (cost_to_waypoint_v1, 
                                     follow_path_waypoints, 
                                     calculate_nominal_path_short,
                                     follow_path_order)

from hs4dpp_main_utils import calc_heuristic_denom, calc_all_headings
from hs4dpp_path_utils import calc_ball_cost, find_optimal_path_nrmpc

import pdb

'''
Double check the results of the algorithm
Can also be used for an apples to oranges comparision between the two


'''

def double_check_results(nom_file, astar_file):
    with open(nom_file, 'rb') as f:
        nom_waypoints = np.load(f)

    uuv = UUV()
    env = Env()
    desired_speed = 2.5722     ##meters/second (5 knots)
    time_start_sec = 0.0

    ## Create hotspots of trash and (optinal) visualize
    trash_x_centers = np.array([-250.98494701, -504.8406451, \
                                -132, 345, 876, 423]).reshape(-1,1)
    trash_y_centers = np.array([-508.96243035, -877.89326774, \
                                -687, 354, 120, 348]).reshape(-1,1)
    trash_sigma = [[], []]               
    hotspot_dict = init_hotspots(trash_x_centers, trash_y_centers,
                                 trash_sigma, 20, env.dfunc)
    for xx in range(2000):
        hotspot_dict = update_trash_pos(hotspot_dict, 0, env)

    nom_hotspot_dict = hotspot_dict.copy()
    astar_hotspot_dict = hotspot_dict.copy()

    nom_total_energy, nom_total_time_sec, nom_total_paper_cost = follow_path_order(nom_waypoints, nom_hotspot_dict, uuv, env, desired_speed, time_start_sec, vis_dash=False)

    print ("SWITCHING TO ASTAR")
    with open(astar_file, 'rb') as f:
        astar_waypoints = np.load(f)

    uuv = UUV()
    env = Env()
    astar_total_energy, astar_total_time_sec, astar_total_paper_cost = follow_path_order(astar_waypoints, astar_hotspot_dict, uuv, env, desired_speed, time_start_sec, vis_dash=False)

    print ("NOMINAL RESULTS")
    print ("total energy    : ", nom_total_energy)
    print ("total time (sec): ", nom_total_time_sec)
    print ("total cost eq   : ", nom_total_paper_cost)

    print ("ASTAR RESULTS")
    print ("total energy    : ", astar_total_energy)
    print ("total time (sec): ", astar_total_time_sec)
    print ("total cost eq   : ", astar_total_paper_cost)

if __name__ == '__main__':
    # nom_file = './nom_path_check.npy'
    # astar_file = './astar_path_check.npy'

    nom_file = './nom_path_seed.npy'
    astar_file = './astar_path_seed.npy'
    double_check_results(nom_file, astar_file)