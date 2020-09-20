from math import atan2, sin, cos, acos
from copy import deepcopy
from datetime import datetime
import csv

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


def init_hotspot_dict(env):
    ## Create hotspots of trash and (optinal) visualize
    # trash_x_centers = np.array([-250.98494701, -504.8406451, \
                                # -132, 345, 876, 423]).reshape(-1,1)
    # trash_y_centers = np.array([-508.96243035, -877.89326774, \
                                # -687, 354, 120, 348]).reshape(-1,1)

    np.random.seed(29435)
    trash_x_centers = np.random.randint(env.xbound[0], env.xbound[1], 6).reshape(-1,1)
    trash_y_centers = np.random.randint(env.ybound[0], env.ybound[1], 6).reshape(-1,1)
    ##Use this to auto generate gaussian trash distributions:
    trash_sigma = [[], []]               
    hotspot_dict = init_hotspots(trash_x_centers, trash_y_centers,
                                 trash_sigma, 20, env.dfunc)
    for xx in range(1000):
        hotspot_dict = update_trash_pos(hotspot_dict, 0, env)

    return hotspot_dict


def create_nom_cost_matrix(hspt1, hspt2, hotspot_dict, desired_speed, wpt_spacing, env, uuv, vis=False):
    '''
    
    '''

    time_hrs = np.arange(0, 72, 0.5)
    energy_cost_matrix = []
    time_cost_matrix = []
    est_cost_matrix = []

    # heuristic_denom = calc_heuristic_denom(env, desired_speed)
    # max_num_epochs = 45000
    # uuv_end_threshold = 10
    # astar_goal_dist = 20

    # for t in range(len(time_hrs)):

    t = 0
    ##Update the trash positions up to this time
    if t>0:
        print ("Updating trash positions for t: {0}".format(t))
        for diff_sec in range(int(time_hrs[t-1]*3600), int(time_hrs[t]*3600), 10):
            diff_hrs = diff_sec/3600
            hotspot_dict = update_trash_pos(hotspot_dict, diff_hrs, env)

    ## Get convex hull of each hotspot plus some buffer
    convexhull_a = sg.MultiPoint(hotspot_dict[hspt1][-1][:, 0:2]).convex_hull
    buffer_a = convexhull_a.buffer(5)
    ax, ay = buffer_a.exterior.coords.xy

    ## Get the convex hull of the corresponding hotspot plus some buffer
    convexhull_b = sg.MultiPoint(hotspot_dict[hspt2][-1][:, 0:2]).convex_hull
    buffer_b = convexhull_b.buffer(5)
    bx, by = buffer_b.exterior.coords.xy

    ##Calculate the closest points between the hotspots
    pt1, pt2 = nearest_points(buffer_a, buffer_b)
    # pt1_depth = env.dfunc([pt1.x, pt1.y])[0]
    # pt2_depth = env.dfunc([pt2.x, pt2.y])[0]
    pt1_depth = 0.0
    pt2_depth = 0.0
    pt1_3d = np.array([pt1.x, pt1.y, pt1_depth])
    pt2_3d = np.array([pt2.x, pt2.y, pt2_depth])
    print ("hotspot {0}, {1}".format(hspt1, hspt2))
    print (pt1_3d)
    print (pt2_3d)
    print ("pt1: ", env.uvfunc([0, 0, pt1.x, pt1.y]))
    print ("pt2: ", env.uvfunc([0, 0, pt2.x, pt2.y]))
    pdb.set_trace()

    ## Calculate path and cost of traveling that path
    ## Append to cost matricies
    nom_path = calculate_nominal_path_short(pt1_3d, pt2_3d, wpt_spacing, env)
    nom_energy, nom_time_sec, nom_est_cost, none = follow_path_waypoints(nom_path,
                                                            pt1_3d,
                                                            uuv,
                                                            env,
                                                            desired_speed,
                                                            time_hrs[t],
                                                            *[False])
    print ("NOM ENERGY: ", nom_energy)
    print ("NOM TIME: ", nom_time_sec)
    print ("NOM_EST_COST: ", nom_est_cost)

    energy_cost_matrix.append(nom_energy)
    time_cost_matrix.append(nom_time_sec)
    est_cost_matrix.append(nom_est_cost) 

    # energy_cost_matrix = np.array(energy_cost_matrix)
    # time_cost_matrix = np.array(time_cost_matrix)
    # est_cost_matrix = np.array(est_cost_matrix)

    ##visualize
    ##TODO: standardize x axis height?
    if vis == True:
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
        fig.suptitle('Comparing path costs for wpt1{0} to wpt2{1}'.format(pt1_3d, pt2_3d))
        ax1.plot(time_hrs, energy_cost_matrix, 'o-')
        ax1.set_ylabel('Energy Cost')
        ax2.plot(time_hrs, time_cost_matrix, 'o-')
        ax2.set_ylabel('Time cost (sec)')
        ax3.plot(time_hrs, est_cost_matrix, 'o-')
        ax3.set_ylabel('Est Cost')
        ax3.set_xlabel('time (hrs)')
        plt.savefig('./santa_monica_costs/surface_hspt{0}_to_{1}_cost_graphs'.format(hspt1, hspt2))
        # plt.show()

        ##save figure here



    return energy_cost_matrix, time_cost_matrix, est_cost_matrix


def create_astar_cost_matrix(hspt1, hspt2, hotspot_dict, desired_speed, wpt_spacing, env, uuv, vis=False):
    '''
    
    '''

    time_hrs = np.arange(0, 72, 0.5)
    energy_cost_matrix = []
    time_cost_matrix = []
    est_cost_matrix = []

    heuristic_denom = calc_heuristic_denom(env, desired_speed)
    max_num_epochs = 45000
    uuv_end_threshold = 10
    astar_goal_dist = 1000
    ball_cart_pos, ball_sphere_headings = calc_all_headings(45)

    savetime = datetime.now().strftime('%Y%m%d%H%M%S')

    # for t in range(len(time_hrs)):

    t = 0

    ##Update the trash positions up to this time
    if t>0:
        print ("Updating trash positions for t: {0}".format(t))
        for diff_sec in range(int(time_hrs[t-1]*3600), int(time_hrs[t]*3600), 10):
            diff_hrs = diff_sec/3600
            hotspot_dict = update_trash_pos(hotspot_dict, diff_hrs, env)

    ## Get convex hull of each hotspot plus some buffer
    convexhull_a = sg.MultiPoint(hotspot_dict[hspt1][-1][:, 0:2]).convex_hull
    buffer_a = convexhull_a.buffer(5)
    ax, ay = buffer_a.exterior.coords.xy

    ## Get the convex hull of the corresponding hotspot plus some buffer
    convexhull_b = sg.MultiPoint(hotspot_dict[hspt2][-1][:, 0:2]).convex_hull
    buffer_b = convexhull_b.buffer(5)
    bx, by = buffer_b.exterior.coords.xy

    ##Calculate the closest points between the hotspots
    pt1, pt2 = nearest_points(buffer_a, buffer_b)
    # pt1_depth = env.dfunc([pt1.x, pt1.y])[0]
    # pt2_depth = env.dfunc([pt2.x, pt2.y])[0]
    pt1_depth = 0.0
    pt2_depth = 0.0
    pt1_3d = np.array([pt1.x, pt1.y, pt1_depth])
    pt2_3d = np.array([pt2.x, pt2.y, pt2_depth])

    ## Calculate path and cost of traveling that path
    ## Append to cost matricies
    # nom_path = calculate_nominal_path_short(pt1_3d, pt2_3d, wpt_spacing, env)
    # nom_energy, nom_time_sec, nom_est_cost, none = follow_path_waypoints(nom_path,
    #                                                         pt1_3d,
    #                                                         uuv,
    #                                                         env,
    #                                                         desired_speed,
    #                                                         time_hrs[t],
    #                                                         *[False])

    astar_total_time_sec = time_hrs[t]/3600
    last_uuv_pos = deepcopy(uuv.pos)
    astr_eq_cost, astr_time_cost_sec, astr_energy_cost, astr_path = find_optimal_path_nrmpc(astar_total_time_sec, pt1_3d, pt2_3d, ball_cart_pos, uuv, env, heuristic_denom, desired_speed, astar_goal_dist, uuv_end_threshold, max_num_epochs, *[False])

    uuv.pos = deepcopy(last_uuv_pos)
    acheck_energy, acheck_time_sec, acheck_paper_cost = follow_path_order([astr_path], hotspot_dict, uuv, env, desired_speed, astar_total_time_sec, **{'uuv_path_state': 'following'})


    energy_cost_matrix.append(acheck_energy)
    time_cost_matrix.append(acheck_time_sec)
    est_cost_matrix.append(acheck_paper_cost) 

    results_txt_file = open('astar_paths_sn_s1_k3.txt', "a")
    results_txt_file.write("\nHOTSPOT{0} TO HOTSPOT{1} AT TIME_HRS{2}\n".format(hspt1,hspt2,time_hrs[t]))
    results_txt_file.write("Energy: {0}".format(energy_cost_matrix))
    results_txt_file.write("Time: {0}".format(time_cost_matrix))
    results_txt_file.write("Est : {0}".format(est_cost_matrix))
    results_txt_file.write(str(astr_path))
    results_txt_file.close()

    # energy_cost_matrix = np.array(energy_cost_matrix)
    # time_cost_matrix = np.array(time_cost_matrix)
    # est_cost_matrix = np.array(est_cost_matrix)

    ##visualize
    ##TODO: standardize x axis height?
    # if vis == True:
    #     fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
    #     fig.suptitle('Comparing path costs for wpt1{0} to wpt2{1}'.format(pt1_3d, pt2_3d))
    #     ax1.plot(time_hrs, energy_cost_matrix, 'o-')
    #     ax1.set_ylabel('Energy Cost')
    #     ax2.plot(time_hrs, time_cost_matrix, 'o-')
    #     ax2.set_ylabel('Time cost (sec)')
    #     ax3.plot(time_hrs, est_cost_matrix, 'o-')
    #     ax3.set_ylabel('Est Cost')
    #     ax3.set_xlabel('time (hrs)')
    #     plt.savefig('./santa_monica_costs/astar_hspt{0}_to_{1}_cost_graphs'.format(hspt1, hspt2))
    #     # plt.show()

        ##save figure here

    return energy_cost_matrix, time_cost_matrix, est_cost_matrix

'''

def vis(env, hotspot_dict):
    # grid_spacing = 100
    grid_spacing = 500
    time_hrs = np.arange(0, 72, 0.5)
    depths = grid_data(env.dfunc, env.xbound, env.ybound, grid_spacing, [], [])
    w,h = np.meshgrid(np.linspace(-env.width/2, env.width/2, depths.shape[1], endpoint=True),
                  np.linspace(-env.height/2, env.height/2, depths.shape[0], endpoint=True)) 

    for t in range(len(time_hrs)):

        ##Update the trash positions up to this time
        if t>0:
            print ("Updating trash positions for t: {0}".format(t))
            for diff_sec in range(int(time_hrs[t-1]*3600), int(time_hrs[t]*3600), 10):
                diff_hrs = diff_sec/3600
                hotspot_dict = update_trash_pos(hotspot_dict, diff_hrs, env)

        fig, ax1 = plt.subplots()
        ax1.set_xlim(env.xbound[0], env.xbound[1])
        ax1.set_ylim(env.ybound[0], env.ybound[1])
        ax1.set_title("Time={0} hrs".format(time_hrs[t]))

        ##Plot all the currents here at this time
        u = grid_data(env.ufunc, env.xbound, env.ybound, grid_spacing, depths, [time_hrs[t]])
        v = grid_data(env.vfunc, env.xbound, env.ybound, grid_spacing, depths, [time_hrs[t]])
        uv = grid_data(env.uvfunc, env.xbound, env.ybound, grid_spacing, depths, [time_hrs[t]])

        vmin = np.min(uv)
        vmax = np.max(uv)

        ax1.scatter(w,h, color='b', s=15)
        im = ax1.quiver(w,h, u, v, uv)

        fig.colorbar(im, orientation='horizontal')
        # im.set_clim(0, 0.15)
        im.set_clim(vmin, vmax)

        for hspt1 in range(0,6):

            ## Get convex hull of each hotspot plus some buffer
            convexhull_a = sg.MultiPoint(hotspot_dict[hspt1][-1][:, 0:2]).convex_hull
            buffer_a = convexhull_a.buffer(5)
            ax, ay = buffer_a.exterior.coords.xy

            ax1.plot(ax, ay, "k-", linewidth = '5')
            ax1.text(ax[0], ay[0], "{0}".format(hspt1), fontsize="10", color='green')

        plt.savefig('./check_blobs_catalina/{0}'.format(t))
'''

if __name__ == '__main__':
    env = Env(1, 1, "san_nicolas") ##This contains (ufunc, vfunc, width, height, u_boost, v_boost)
    uuv = UUV()

    #For 4D path planning
    # desired_speed = 2.5722     ##meters/second (5 knots)
    desired_speed = 1.5            ##meters/sec (3 knots)
    # desired_speed = 0.7         ##meters/sec (1.5 knot)
    wpt_spacing = 1000

    time_hrs = np.arange(0, 72, 0.5)

    # hspt1 = 1
    # hspt2 = 0

    # hotspot_dict = init_hotspot_dict(env)
    # vis(env, hotspot_dict)
    
    # energy_matrix, time_cost_sec_matrix, est_cost_matrix = create_nom_cost_matrix(hspt1, hspt2, hotspot_dict, desired_speed, wpt_spacing, env, uuv)

    num_hotspots = 6
    for hspt1 in range(0, num_hotspots):
        for hspt2 in range(0, num_hotspots):
            if hspt1 == hspt2:
                continue
            print ("analyzing hotspot {0} to {1}".format(hspt1, hspt2))

            hotspot_dict = init_hotspot_dict(env)
            energy_matrix, time_cost_sec_matrix, est_cost_matrix = create_nom_cost_matrix(hspt1, hspt2, hotspot_dict, desired_speed, wpt_spacing, env, uuv)
            # energy_matrix, time_cost_sec_matrix, est_cost_matrix = create_astar_cost_matrix(hspt1, hspt2, hotspot_dict, desired_speed, wpt_spacing, env, uuv)

            # print ("energy_matrix: ", energy_matrix)
            # print ("time_matrix: ", time_cost_sec_matrix)
            # print ("est_matrix: ", est_cost_matrix)
            # pdb.set_trace()

            ##save results to file here?
            title_row = ["Hspt{0}_to_Hspt{1}".format(hspt1, hspt2)] + time_hrs.tolist()
            energy_row = ["Energy_cost"] + energy_matrix
            time_row = ["Time_sec_cost"] + time_cost_sec_matrix
            est_row = ["Est_cost"] + est_cost_matrix
            all_costs = [energy_row, time_row, est_row]

            # with open('surface3_hspt{0}_energy_costs.csv'.format(hspt1), 'a') as csvfile:
            #     csvwriter = csv.writer(csvfile)
            #     csvwriter.writerow(title_row)
            #     csvwriter.writerow(energy_row)

            # with open('surface3_hspt{0}_time_costs.csv'.format(hspt1), 'a') as csvfile:
            #     csvwriter = csv.writer(csvfile)
            #     csvwriter.writerow(title_row)
            #     csvwriter.writerow(time_row)

            # with open('surface3_hspt{0}_est_costs.csv'.format(hspt1), 'a') as csvfile:
            #     csvwriter = csv.writer(csvfile)
            #     csvwriter.writerow(title_row)
            #     csvwriter.writerow(est_row)
                
            # with open('astar1000_surface1_knots3_hspt{0}_costs.csv'.format(hspt1), 'a') as csvfile:
            #     csvwriter = csv.writer(csvfile)
            #     csvwriter.writerow(title_row)
            #     csvwriter.writerows(all_costs)