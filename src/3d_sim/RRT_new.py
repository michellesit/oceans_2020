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
                                     follow_path_waypoints_v2, 
                                     calculate_nominal_path_short,
                                     follow_path_order)
from trash_utils.visualize import (visualize_multi_current)

from trash_utils.finder_utils import get_current_block, get_depth_block, lat_lon_to_xy, grid_data, get_multi_current_block


from hs4dpp_main_utils import calc_heuristic_denom, calc_all_headings
from hs4dpp_path_utils import calc_ball_cost, find_optimal_path_nrmpc

import pdb

import pickle

import rospkg
from scipy.io import netcdf
from trash_utils.haversine_dist import haversine


class Point():
    def __init__(self, x, y, z, parent=None):
        self.x = x
        self.y = y
        self.z = z
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0

    def set_x(self, new_x):
        self.x = new_x

    def set_y(self, new_y):
        self.y = new_y

    def set_z(self, new_z):
        self.z = new_z

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_z(self):
        return self.z

    def set_parent(self, parent_pt):
        self.parent = parent_pt
        return

    def get_pos(self):
        return [self.x, self.y, self.z]


class RRT_AStar():
    def __init__(self):
        self.current_pos = np.empty((0,2))

    def update_pos(self, new_pos):
        self.current_pos = new_pos

    def get_current_pos(self):
        return self.current_pos



    def calculate_ball(self, p, radius):
        ball_pos_x = np.ones((3, 3))*p.get_x()
        ball_pos_y = np.ones((3, 3))*p.get_y()
        # ball_pos_z = np.ones((3, 3,3))*p.get_z()

        ball_pos_x[:,0] -= radius
        ball_pos_x[:,2] += radius
        
        ball_pos_y[0,:] += radius
        ball_pos_y[2,:] -= radius

        # ball_pos_z[:,:,0] += radius
        # ball_pos_z[:,:,2] -= radius

        flatx = ball_pos_x.flatten()
        flatx = np.delete(flatx, (4))
        flaty = ball_pos_y.flatten()
        flaty = np.delete(flaty, (4))

        remove = []
        keep = []
        for i in range(len(flatx)):
            if (flatx[i] < env.xbound[0]) or (flatx[i] > env.xbound[1] ) or (flaty[i] < env.ybound[0]) or (flaty[i] > env.ybound[1]):
                remove.append(i)
            else:
                keep.append(i)

        flatx = flatx[keep]
        flaty = flaty[keep]

        all_ball = np.array(zip(flatx, flaty))
        # all_ball = np.delete(all_ball, (4), axis=0)
        # all_ball[all_ball[:,0]>env.width] = env.width

        # fig = plt.figure()
        # plt.plot(p[0], p[1])
        # plt.plot(all_ball[:,0], all_ball[:,1], 'o')
        # plt.show()

        return all_ball




    def step_to_point(self, point1, point2):
        step_size = 100 ##m
        step_iters = 2000
        step = 0

        start_root = Point(point1[0], point1[1], point1[2], None)
        start_root.g = start_root.h = start_root.f = 0
        end_root = Point(point2[0], point2[1], point2[2], None)
        end_root.g = end_root.h = end_root.f = 0

        open_loc = []
        closed_loc = []

        open_loc.append(start_root)

        largest_dist = abs(np.linalg.norm([point1 - point2]))

        while len(open_loc) > 0 and step<step_iters:
            current_pos = open_loc[0]
            current_idx = 0
            for idx, item in enumerate(open_loc):
                if item.f <= current_pos.f:
                    current_pos = item
                    current_idx = idx

            ##Take current_pos off the open list, add to closed list
            open_loc.pop(current_idx)
            closed_loc.append(current_pos)

            # print ("end_root: ", end_root.get_pos())
            print ("dist to end goal: ", np.linalg.norm([current_pos.get_pos() - point2]))
            # pdb.set_trace()
            if abs(np.linalg.norm([current_pos.get_pos() - point2])) < step_size:
                ans_path = []
                ans_energy = 0
                ans_time = 0
                current = current_pos
                # pdb.set_trace()

                # fig = plt.figure()
                # open_loc = np.array(open_loc)
                # closed_loc = np.array(closed_loc)
                # plt.plot(open_loc[:,0], open_loc[:,1], 'o')
                # plt.plot(closed_loc[:,0], closed_loc[:,1], 'o')
                # plt.plot()


                while current is not None:
                    ans_path.append(current.get_pos())
                    ans_energy += current.g
                    ans_time += current.h
                    current = current.parent

                # pdb.set_trace()

                # print ("current pos: ", current.get_pos())
                # print ("current g  : ", current.g)
                # print ("current h  : ", current.h)
                # print ("current parent: ", current.parent.get_pos())
                # print ("ans_energy: ", ans_energy)
                # print ("ans_time: ", ans_time)
                # print ('ans_path: ', ans_path)

                return ans_path[::-1], ans_energy, ans_time

            ##Generate children
            ball = self.calculate_ball(current_pos, step_size)
            ball_z = np.zeros((ball.shape[0], ball.shape[1])).flatten()

            children_pts = []
            for b in range(len(ball)):
                children_pts.append(Point(ball[b][0], ball[b][1], ball_z[b], current_pos))

            ##Loop through the children:
            donotadd_closed_list = False
            donotadd_open_list = False
            for c in children_pts:

                ##Check if c position already exists in the closed list
                for closed_child in closed_loc:
                    if c.get_pos() == closed_child.get_pos():
                        donotadd_closed_list = True

                if donotadd_closed_list == False:
                    toNode = np.vstack([current_pos.get_pos(), c.get_pos()])
                    energy, time_sec, est_cost, ig = follow_path_waypoints_v2(toNode,
                                                                        env,
                                                                        desired_speed,
                                                                        sample_sec,
                                                                        *[False])

                    c.g = energy         ###
                    c.h = time_sec       ###
                    # c.f = c.g + c.h + np.linalg.norm([current_pos.get_pos() - point2])
                    c.f = energy + abs(np.linalg.norm([current_pos.get_pos() - point2]))*0.8

                ##if this child is already in the open list:
                for open_node in open_loc:
                    if c.get_pos() == open_node.get_pos():
                        donotadd_open_list = True

                if donotadd_open_list == False:
                    open_loc.append(c)

            step += 1
            print ('step: {0} / {1}'.format(step, step_iters))

        print ("DID NOT FIND A SOLUTION")
        return [], np.inf, np.inf



if __name__ == '__main__':
    rospack = rospkg.RosPack()
    trash_finder_path = rospack.get_path("trash_finder")
    data_path = rospack.get_path("data_files")

    config = pickle.load(open(trash_finder_path + '/config/demo_configs.p', 'rb'))

    ##hydrodynamic current
    current_path = data_path+'/ca_subCA_das_2020010615.nc'
    current_data = netcdf.NetCDFFile(current_path)
    currents1 = config['current_file1']
    currents_path1 = data_path + '/' + currents1
    currents2 = config['current_file2']
    currents_path2 = data_path + '/' + currents2
    depth = current_data.variables['depth'][:].copy()
    current_data.close()

    ##bathymetry model
    topo = config['topo_file']
    topo_path = data_path + '/' + topo

    all_locations = pickle.load( open(trash_finder_path+"/config/locations.p", "rb"))

    place_bbox = np.array([[33.517, -119.900],
                            [33.517, -119.786],
                            [33.428, -119.786],
                            [33.428, -119.900]])



    env = Env(1,1)
    uuv = UUV()

    RT = RRT_AStar()
    # P = Path()
    # PT = Point()
    # ufunc,vfunc,dfunc,uvfunc = load_currents()
    # desired_speed = 1.5
    desired_speed = 2.5722
    sample_hr = 0

    #get waypoints
    np.random.seed(214323)
    # hotspot_dict = init_hotspot()

    h0 = np.array([0,0,0])
    h1 = np.array([ 2413.17937189, -1454.27348944,     0.        ] )
    h2 = np.array([-4955.77214741, -1807.45178534,     0.        ] )
    h3 = np.array([ 2431.56786602,  -895.66810652,     0.        ] )
    h4 = np.array([-2301.39176315,  3674.90015933,     0.        ] )
    h5 = np.array([-1590.5154935 ,   22.5489957  ,     0.        ] )
    h6 = np.array([  104.16639771, -4009.83609744,     0.        ] )
    all_hotspot = [h0, h1, h2, h3, h4, h5, h6]

    # visualize_multi_current(place_bbox, currents_path1, currents_path2, topo_path, h1, h3)


    all_energy_costs = np.ones((len(all_hotspot), len(all_hotspot)))*9999999999999999999999999
    all_time_costs = np.ones((len(all_hotspot), len(all_hotspot)))*9999999999999999999999999

    for hspt1 in range(len(all_hotspot)):
        # for hspt2 in range(len(all_hotspot)):
        for hspt2 in range(4, len(all_hotspot)):
            if hspt1 == hspt2:
                continue

            visualize_multi_current(place_bbox, currents_path1, currents_path2, topo_path, all_hotspot[hspt1], all_hotspot[hspt2])

            sample_sec = sample_hr/3600.0
            nom_path = calculate_nominal_path_short(all_hotspot[hspt1], all_hotspot[hspt2], 1000, env)
            nom_path[:,2] = 0.0
            if len(nom_path) <= 1:
                nom_path = np.vstack((all_hotspot[hspt1], all_hotspot[hspt2]))

            nom_energy, nom_time_sec, nom_est_cost, ignore = follow_path_waypoints_v2(nom_path,
                                                                    env,
                                                                    desired_speed,
                                                                    sample_sec,
                                                                    *[False])

            # visualize_multi_current(place_bbox, currents_path1, currents_path2, topo_path, h0, h3)
            plt.plot(nom_path[:,0], nom_path[:,1])
            # plt.show()

            print ("NOM_ENERGY: ", nom_energy)
            print ("NOM TIME  : ", nom_time_sec)
            print ("NOM EST   : ", nom_est_cost)

            all_energy_costs[hspt1, hspt2] = nom_energy
            all_time_costs[hspt1, hspt2] = nom_time_sec

            # print ('all_energy_costs: ', all_energy_costs)
            # print ("all_time_costs  : ", all_time_costs)
            # pdb.set_trace()



            if nom_energy > 0:
                #break down path into smaller pieces
                optx = np.linspace(all_hotspot[hspt1][0], all_hotspot[hspt2][0], 10).astype(int).reshape(-1,1)
                opty = np.linspace(all_hotspot[hspt1][1], all_hotspot[hspt2][1], 10).astype(int).reshape(-1,1)
                optz = np.zeros((len(optx),1))
                waypoints = np.hstack((optx, opty, optz))

                plt.plot(waypoints[:,0], waypoints[:,1], 'o')

                total_ref_energy = 0
                total_ref_time = 0
                for w in range(len(waypoints)-1):
                    ##get the amount of energy you need to beat for this part of the path
                    all_w = np.vstack([waypoints[w], waypoints[w+1]])
                    ref_energy, ref_time_sec, ref_est_cost, ref_path = follow_path_waypoints_v2(
                                                                    all_w,
                                                                    env,
                                                                    desired_speed,
                                                                    sample_sec,
                                                                    *[False])

                    print ("pt1: ", waypoints[w])
                    print ("pt2: ", waypoints[w+1])
                    print ("h0, h3: ", h0, h3)
                    print ("REF ENERGY: ", ref_energy)
                    print ("REF TIME  : ", ref_time_sec)
                    print ("REF EST   : ", ref_est_cost)
                    total_ref_energy += ref_energy
                    total_ref_time += ref_time_sec
                    plt.plot(ref_path[:,0], ref_path[:,1])

                    print ("total_ref_energy: ", total_ref_energy)
                    print ("NOM_ENERGY: ", nom_energy)
                    print ("NOM TIME  : ", nom_time_sec)
                    print ("NOM EST   : ", nom_est_cost)
                    
                # plt.show()

                    total_astar_energy = 0.0
                    total_astar_time = 0.0
                    if ref_energy > 50:
                        print ('Looking for more efficient paths')


                        astar_path, astar_energy, astar_time = RT.step_to_point(waypoints[w], waypoints[w+1])
                        print ("we have a solution!")
                        print ("astar energy: ", astar_energy)
                        print ("astar time  : ", astar_time)
                        print ("path        : ", astar_path)
                        total_astar_energy += astar_energy
                        total_astar_time   += astar_time

                        if len(astar_path) >0:
                            astar_path = np.array(astar_path)
                            # plt.clf()
                            # fig = plt.figure()
                            plt.plot(astar_path[:,0], astar_path[:,1])
                            # plt.plot(all_w[:,0], all_w[:,1])
                            # plt.show()

                        if ref_energy > astar_energy:
                            print ("WE FOUND A THING!! VERY EXCITING")
                            # pdb.set_trace()
                        else:
                            print ("ref is better, moving onto next point")
                            print ("REF ENERGY: ", ref_energy)
                            print ("REF TIME  : ", ref_time_sec)
                            print ("astar energy: ", astar_energy)
                            print ("astar time  : ", astar_time)
                            # pdb.set_trace()

            print ("COMPARE ALL THREE RESULTS NOW")
            print ("total_astar_energy: ", total_astar_energy)
            print ("total_astar_time  : ", total_astar_time)
            print ("total_ref_energy  : ", total_ref_energy)
            print ("total_ref_time    : ", total_ref_time)
            print ("NOM_ENERGY        : ", nom_energy)
            print ("NOM TIME          : ", nom_time_sec)

            plt.show()
            pdb.set_trace()

            ##Save all the info that we got
            results_txt_file = open('RRT_solutions.txt', "a")
            results_txt_file.write("\nHOTSPOT{0} TO HOTSPOT{1} AT TIME_HRS{2}\n".format(hspt1,hspt2,sample_hr))
            results_txt_file.write("\nNOM Energy: {0}".format(nom_energy))
            results_txt_file.write("\nNOM Time  : {0}".format(nom_time_sec))
            results_txt_file.write("\nREF Energy: {0}".format(total_ref_energy))
            results_txt_file.write("\nREF Time  : {0}".format(total_ref_time))
            results_txt_file.write("\nAST Energy: {0}".format(total_astar_energy))
            results_txt_file.write("\nAST Time  : {0}".format(total_astar_time))

            results_txt_file.write("\nNOM Path")
            results_txt_file.write(str(nom_path))
            results_txt_file.write("\nREF Path: ")
            results_txt_file.write(str(ref_path))
            results_txt_file.write("\nAST Path: ")
            results_txt_file.write(str(astar_path))

            results_txt_file.close()


    print ('all_energy_costs: ', all_energy_costs)
    print ("all_time_costs  : ", all_time_costs)
    pdb.set_trace()