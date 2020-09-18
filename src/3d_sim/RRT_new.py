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
from trash_utils.fourD_utils import (cost_to_waypoint_v2, 
                                     follow_path_waypoints_v2, 
                                     calculate_nominal_path_short,
                                     follow_path_order,
                                     currents_score)
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




    def step_to_point(self, point1, point2, sample_sec):
        sample_hrs = sample_sec/3600.0
        # fig, ax1 = plt.subplots()
        # print ("point2", point2)
        # ax1.plot(point1[0], point1[1], '*')
        # ax1.text(point1[0], point1[1], 'START')
        # ax1.plot(point2[0], point2[1], '*')
        # ax1.text(point2[0], point2[1], 'END')

        # min_width = min(point1[0], point2[0])
        # max_width = max(point1[0], point2[0])
        # min_height = min(point1[1], point2[1])
        # max_height = max(point1[1], point2[1])

        # width = abs(max_width - min_width)
        # height = abs(max_height - min_height)
        # xwidth = [min_width, max_width]
        # yheight = [min_height, max_height]

        # ufunc, vfunc, uvfunc = get_multi_current_block(place_bbox, currents_path1, currents_path2)
        # d_area, dfunc = get_depth_block(place_bbox, topo_path)
        # depths = grid_data(dfunc, xwidth, yheight, 100, [], [])
        # depths = np.ones((depths.shape[0], depths.shape[1]))*0
        # depths_flat = depths.flatten()

        # w,h = np.meshgrid(np.linspace(xwidth[0], xwidth[1], depths.shape[1], endpoint=True),
        #                   np.linspace(yheight[0], yheight[1], depths.shape[0], endpoint=True))    

        # u = grid_data(ufunc, xwidth, yheight, 500, depths, [sample_hrs])
        # v = grid_data(vfunc, xwidth, yheight, 500, depths, [sample_hrs])
        # uv = grid_data(uvfunc, xwidth, yheight, 500, depths, [sample_hrs])

        # vmin = np.min(uv)
        # vmax = np.max(uv)

        # im1 = ax1.quiver(w,h, u, v, uv)


        step_size = 100 ##m
        step_iters = 100
        step = 0

        start_root = Point(point1[0], point1[1], point1[2], None)
        start_root.g = start_root.h = start_root.f = 0
        end_root = Point(point2[0], point2[1], point2[2], None)
        end_root.g = end_root.h = end_root.f = 0

        open_loc = []
        closed_loc = []
        open_loc.append(start_root)

        largest_dist = abs(np.linalg.norm([point1 - point2]))
        alpha = 0.8

        while len(open_loc) > 0 and step<step_iters:
            fig, ax1 = plt.subplots(figsize=(25, 25))
            print ("point2", point2)
            ax1.plot(point1[0], point1[1], '*')
            ax1.text(point1[0]+5, point1[1]+5, 'START')
            ax1.plot(point2[0], point2[1], '*')
            ax1.text(point2[0]+5, point2[1]+5, 'END')

            min_width = min(point1[0], point2[0])
            max_width = max(point1[0], point2[0])
            min_height = min(point1[1], point2[1])
            max_height = max(point1[1], point2[1])

            width = abs(max_width - min_width)
            height = abs(max_height - min_height)
            xwidth = [min_width, max_width]
            yheight = [min_height, max_height]

            ufunc, vfunc, uvfunc = get_multi_current_block(place_bbox, currents_path1, currents_path2)
            d_area, dfunc = get_depth_block(place_bbox, topo_path)
            depths = grid_data(dfunc, xwidth, yheight, 100, [], [])
            depths = np.ones((depths.shape[0], depths.shape[1]))*0
            depths_flat = depths.flatten()

            w,h = np.meshgrid(np.linspace(xwidth[0], xwidth[1], depths.shape[1], endpoint=True),
                              np.linspace(yheight[0], yheight[1], depths.shape[0], endpoint=True))    

            u = grid_data(ufunc, xwidth, yheight, 500, depths, [sample_hrs])
            v = grid_data(vfunc, xwidth, yheight, 500, depths, [sample_hrs])
            uv = grid_data(uvfunc, xwidth, yheight, 500, depths, [sample_hrs])

            vmin = np.min(uv)
            vmax = np.max(uv)

            im1 = ax1.quiver(w,h, u, v, uv)



            current_pos = open_loc[0]
            current_idx = 0
            for idx, item in enumerate(open_loc):
                print ('item.f    : ', item.f)
                print ("current.f : ", current_pos.f)
                if item.f >= current_pos.f:
                    current_pos = deepcopy(item)
                    current_idx = deepcopy(idx)

                    top_score = deepcopy(item.f)
                    next_pos = deepcopy(item)

                if item.get_pos() == end_root.get_pos():
                    current_pos = deepcopy(item)
                    current_idx = deepcopy(idx)
                    break

            ##Take current_pos off the open list, add to closed list
            open_loc.pop(current_idx)
            closed_loc.append(current_pos)
            print ("TOP SCORE: ", top_score)
            print ("next_pos : ", next_pos.get_pos())

            # ax1.plot(current_pos.get_x(), current_pos.get_y(), current_pos.get_z(), 'o')

            # print ("end_root: ", end_root.get_pos())
            # print ("dist to end goal: ", np.linalg.norm([current_pos.get_pos() - point2]))
            if abs(np.linalg.norm([current_pos.get_pos() - point2])) < step_size:
                ans_path = []
                # ans_energy = 0
                # ans_time = 0
                current = current_pos

                # fig = plt.figure()
                # open_loc_pts = np.array([c.get_pos() for c in open_loc])
                # closed_loc_pts = np.array([c.get_pos() for c in closed_loc])

                # open_loc = np.array(open_loc)
                # closed_loc = np.array(closed_loc)
                # plt.plot(open_loc_pts[:,0], open_loc_pts[:,1], 'o')
                # plt.plot(closed_loc_pts[:,0], closed_loc_pts[:,1], 'o')

                # plt.plot()


                while current is not None:
                    ans_path.append(current.get_pos())
                    # ans_energy += current.g
                    # ans_time += current.h
                    current = current.parent

                np_ans = np.array(ans_path)
                ax1.plot(np_ans[:,0], np_ans[:,1], '--')
                plt.show()
                # return ans_path[::-1], ans_energy, ans_time
                return ans_path[::-1]

            ##Generate children
            ball = self.calculate_ball(current_pos, step_size)
            ball_z = np.zeros((ball.shape[0], ball.shape[1])).flatten()

            print ('ball: ', ball)

            children_pts = []
            for b in range(len(ball)):
                children_pts.append(Point(ball[b][0], ball[b][1], ball_z[b], current_pos))

            for op in open_loc:
                ax1.plot(op.get_x(), op.get_y(), 'o')
                ax1.text(op.get_x(), op.get_y(), str(op.f))
            for cl in closed_loc:
                ax1.plot(cl.get_x(), cl.get_y(), 'o')
                ax1.text(cl.get_x(), cl.get_y(), 'visited: {0}'.format(cl.f))


            ##Loop through the children:
            max_score = -9999
            for c in children_pts:
                donotadd_closed_list = False
                donotadd_open_list = False
                print ("c: ", c.get_pos())

                ##Check if c position already exists in the closed list
                for closed_child in closed_loc:
                    if c.get_pos() == closed_child.get_pos():
                        print ("THIS VALUE WAS FOUND IN THE CLOSED LIST")
                        print ("c.pos: ", c.get_pos())
                        all_closed = [closed.get_pos() for closed in closed_loc]
                        print ('all_closed: ', np.array(all_closed))
                        # pdb.set_trace()

                        donotadd_closed_list = True

                if (donotadd_closed_list == False):
                    print ("THIS VALUE WAS NOT IN THE CLOSED LOC LIST. CHECKING OPEN LIST NOW")
                    toNode = np.vstack([current_pos.get_pos(), c.get_pos()])
                    ball_score = currents_score(np.array(current_pos.get_pos()), np.array(c.get_pos()), sample_sec, env, largest_dist, alpha)

                    c.f = ball_score

                    ax1.plot(c.get_x(), c.get_y(), 'o')
                    ax1.text(c.get_x()+10, c.get_y()+10, 'new: {0}'.format(c.f))

                    # if c.f > max_score:
                        # top_score = ball_score
                        # next_pos = c.get_pos()

                    # ax1.plot(c.get_x(), c.get_y(), 'o')
                    # ax1.text(c.get_x(), c.get_y(), str(ball_score))
                    # energy, time_sec, est_cost, ig = follow_path_waypoints_v2(toNode,
                    #                                                     env,
                    #                                                     desired_speed,
                    #                                                     sample_sec,
                    #                                                     *[False])

                    # c.g = energy         ###
                    # c.h = time_sec       ###
                    # # c.f = c.g + c.h + np.linalg.norm([current_pos.get_pos() - point2])
                    # c.f = energy + abs(np.linalg.norm([current_pos.get_pos() - point2]))*0.8

                ##if this child is already in the open list:
                for open_node in open_loc:
                    if (c.get_pos() == open_node.get_pos()) and (c.f < open_node.f):
                        print ("THIS VALUE WAS FOUND IN THE OPEN LIST")
                        print ("c.pos: ", c.get_pos())
                        all_open = [op.get_pos() for op in open_loc]
                        print ('all_open: ', np.array(all_open))
                        # pdb.set_trace()

                        donotadd_open_list = True

                    if (c.get_pos() == open_node.get_pos()) and (c.f > open_node.f):
                        print ("THIS VALUE WAS FOUND IN THE OPEN LIST BUT SWITCHING OUT F VALUES")
                        print ("c.pos: ", c.get_pos())
                        print ("c.f  : ", c.f)
                        print ("old.f: ", open_node.f)
                        
                        replace_idx = open_loc.index(open_node)
                        open_loc[replace_idx] = c

                        donotadd_open_list = True

                if donotadd_open_list == False:
                    print ("THIS VALUE WAS NOT FOUND IN THE OPEN LIST. ADDING TO THE OPEN LIST")
                    open_loc.append(c)

            step += 1

            ax1.text(next_pos.get_x()+15, next_pos.get_y()+15, "Current pos")
            # plt.savefig('./test_ball/{0}.png'.format(step))
            plt.show()
            print ('step: {0} / {1}'.format(step, step_iters))
            # pdb.set_trace()

        print ("DID NOT FIND A SOLUTION")
        # plt.show()
        # return [], np.inf, np.inf
        return []


def calc_nom_soln():
    #get waypoints
    np.random.seed(214323)

    h0 = np.array([0,0,0])
    h1 = np.array([ 2413.17937189, -1454.27348944,     0.        ] )
    h2 = np.array([-4955.77214741, -1807.45178534,     0.        ] )
    h3 = np.array([ 2431.56786602,  -895.66810652,     0.        ] )
    h4 = np.array([-2301.39176315,  3674.90015933,     0.        ] )
    h5 = np.array([-1590.5154935 ,   22.5489957  ,     0.        ] )
    h6 = np.array([  104.16639771, -4009.83609744,     0.        ] )
    all_hotspot = [h0, h1, h2, h3, h4, h5, h6]
    np_all_hotspots = np.array(all_hotspot)

    time_hrs = np.arange(0, 72, 1)

    for hspt1 in range(1, len(all_hotspot)):
        for hspt2 in range(len(all_hotspot)):
            if hspt1 == hspt2:
                continue

            if (hspt1 == 1) and (hspt2 < 4):
                continue

            all_energy_costs = []
            all_time_costs   = []

            for t_hr in time_hrs:
                sample_hrs = t_hr

                print ("Traveling from HTSP{0} to HTSP{1} at TIME{2}".format(hspt1, hspt2, sample_hrs))

                sample_sec = sample_hrs*3600.0
                nom_path = calculate_nominal_path_short(all_hotspot[hspt1], all_hotspot[hspt2], 1000, env)
                nom_path[:,2] = 0.0
                if len(nom_path) <= 1:
                    nom_path = np.vstack((all_hotspot[hspt1], all_hotspot[hspt2]))

                nom_energy, nom_time_sec, nom_est_cost, ignore = follow_path_waypoints_v2(nom_path,
                                                                        env,
                                                                        desired_speed,
                                                                        sample_sec,
                                                                        *[False])

                ##Visualize currents at this step
                fig, ax1 = plt.subplots()

                # ax1.plot(all_hotspot[hspt1][0], all_hotspot[hspt1][1], 'o')
                # ax1.plot(all_hotspot[hspt2][0], all_hotspot[hspt2][1], 'o')
                ax1.plot(np_all_hotspots[:, 0], np_all_hotspots[:,1], 'o')

                ax1.text(h0[0], h0[1], "H0", fontsize='20')
                ax1.text(h1[0], h1[1], "H1", fontsize='20')
                ax1.text(h2[0], h2[1], "H2", fontsize='20')
                ax1.text(h3[0], h3[1], "H3", fontsize='20')
                ax1.text(h4[0], h4[1], "H4", fontsize='20')
                ax1.text(h5[0], h5[1], "H5", fontsize='20')
                ax1.text(h6[0], h6[1], "H6", fontsize='20')

                min_width = min(all_hotspot[hspt1][0], all_hotspot[hspt2][0]) - 100
                max_width = max(all_hotspot[hspt1][0], all_hotspot[hspt2][0]) + 100
                min_height = min(all_hotspot[hspt1][1], all_hotspot[hspt2][1]) - 100
                max_height = max(all_hotspot[hspt1][1], all_hotspot[hspt2][1]) + 100

                width = abs(max_width - min_width)
                height = abs(max_height - min_height)
                xwidth = [min_width, max_width]
                yheight = [min_height, max_height]

                ufunc, vfunc, uvfunc = get_multi_current_block(place_bbox, currents_path1, currents_path2)
                d_area, dfunc = get_depth_block(place_bbox, topo_path)

                depths = grid_data(dfunc, xwidth, yheight, 100, [], [])
                depths = np.ones((depths.shape[0], depths.shape[1]))*0
                depths_flat = depths.flatten()
                time_range = np.arange(0, 72, 1)

                w,h = np.meshgrid(np.linspace(xwidth[0], xwidth[1], depths.shape[1], endpoint=True),
                                  np.linspace(yheight[0], yheight[1], depths.shape[0], endpoint=True))    

                u = grid_data(ufunc, xwidth, yheight, 500, depths, [sample_hrs])
                v = grid_data(vfunc, xwidth, yheight, 500, depths, [sample_hrs])
                uv = grid_data(uvfunc, xwidth, yheight, 500, depths, [sample_hrs])

                vmin = np.min(uv)
                vmax = np.max(uv)

                ax1.set_title('Nominal Path from Hotspot{0} to Hotspot{1} at T={2} Hrs'.format(hspt1, hspt2, sample_hrs))
                im1 = ax1.quiver(w,h, u, v, uv)
                ax1.set_xlim(min_width, max_width)
                ax1.set_ylim(min_height, max_height)
                fig.colorbar(im1)




                ax1.plot(nom_path[:,0], nom_path[:,1])
                plt.savefig('./san_nicolas_costs/rrt_nom_h{0}_to_h{1}_time{2}.png'.format(hspt1, hspt2, sample_hrs))
                plt.clf()
                plt.close('all')

                print ("NOM_ENERGY: ", nom_energy)
                print ("NOM TIME  : ", nom_time_sec)
                print ("NOM EST   : ", nom_est_cost)

                all_energy_costs.append(nom_energy)
                all_time_costs.append(nom_time_sec)

            ###save these to a csv file
            results_txt_file = open('./san_nicolas_costs/rrt_paths_nom_only.txt', "a")
            results_txt_file.write("\nHOTSPOT{0} TO HOTSPOT{1} AT TIME_HRS{2}\n".format(hspt1,hspt2,time_hrs[sample_hrs]))
            results_txt_file.write("\nEnergy: {0}".format(all_energy_costs))
            results_txt_file.write("\nTime: {0}\n".format(all_time_costs))
            results_txt_file.write(str(nom_path))
            results_txt_file.close()            


            title_row = ["Hspt{0}_to_Hspt{1}".format(hspt1, hspt2)] + time_hrs.tolist()
            energy_row = ["Energy_cost"] + all_energy_costs
            time_row = ["Time_sec_cost"] + all_time_costs
            # all_costs = [energy_row, time_row]

            with open('./san_nicolas_costs/rrt_nom_surface1_hspt{0}_energy_time_costs.csv'.format(hspt1), 'a') as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow(title_row)
                csvwriter.writerow(energy_row)
                csvwriter.writerow(time_row)



def cal_ref_astar():
    #get waypoints
    np.random.seed(214323)

    h0 = np.array([0,0,0])
    h1 = np.array([ 2413.17937189, -1454.27348944,     0.        ] )
    h2 = np.array([-4955.77214741, -1807.45178534,     0.        ] )
    h3 = np.array([ 2431.56786602,  -895.66810652,     0.        ] )
    h4 = np.array([-2301.39176315,  3674.90015933,     0.        ] )
    h5 = np.array([-1590.5154935 ,   22.5489957  ,     0.        ] )
    h6 = np.array([  104.16639771, -4009.83609744,     0.        ] )
    all_hotspot = [h0, h1, h2, h3, h4, h5, h6]
    np_all_hotspots = np.array(all_hotspot)

    time_hrs = np.arange(0, 1, 1)

    for hspt1 in range(2, len(all_hotspot)):
        for hspt2 in range(len(all_hotspot)):
            if hspt1 == hspt2:
                continue

            # if (hspt1 == 0) and (hspt2 == 1):
            #     continue

            all_ref_energy_costs = []
            all_ref_time_costs   = []
            all_astar_energy_costs = []
            all_astar_time_costs   = []

            for t_hr in time_hrs:
                sample_hrs = t_hr
                print ("Traveling from HTSP{0} to HTSP{1} at TIME{2}".format(hspt1, hspt2, sample_hrs))
                sample_sec = sample_hrs*3600.0

                ####REF PATH
                #break down path into smaller pieces
                optx = np.linspace(all_hotspot[hspt1][0], all_hotspot[hspt2][0], 10).astype(int).reshape(-1,1)
                opty = np.linspace(all_hotspot[hspt1][1], all_hotspot[hspt2][1], 10).astype(int).reshape(-1,1)
                optz = np.zeros((len(optx),1))
                waypoints = np.hstack((optx, opty, optz))

                plt.plot(waypoints[:,0], waypoints[:,1], 'o')

                total_ref_energy = 0.0
                total_ref_time = 0.0
                # total_astar_energy = 0.0
                # total_astar_time = 0.0
                full_ref_path = np.empty((0,3))
                full_astar_path = np.empty((0,3))

                for w in range(len(waypoints)-1):
                    print ("w {0} / {1} waypoints: ".format(w, len(waypoints)))
                    ##get the amount of energy you need to beat for this part of the path
                    all_w = np.vstack([waypoints[w], waypoints[w+1]])
                    ref_energy, ref_time_sec, ref_est_cost, ref_path = follow_path_waypoints_v2(
                                                                    all_w,
                                                                    env,
                                                                    desired_speed,
                                                                    sample_sec,
                                                                    *[False])

                    total_ref_energy += ref_energy
                    total_ref_time += ref_time_sec
                    # full_ref_path.append(ref_path)
                    full_ref_path = np.vstack((full_ref_path, ref_path))
                    plt.plot(ref_path[:,0], ref_path[:,1])

                    ##ASTAR PATH
                    # print ('Looking for more efficient paths')
                    astar_path = RT.step_to_point(waypoints[w], waypoints[w+1], sample_sec)
                    ##Get actual cost of path:
                    # follow_path_waypoints_v2(astar_path, env, desired_speed, sample_sec, *[False])
                    # total_astar_energy += astar_energy
                    # total_astar_time   += astar_time

                    if len(astar_path) >0:
                        astar_path = np.array(astar_path)
                        # full_astar_path.append(astar_path)
                        full_astar_path = np.vstack((full_astar_path, astar_path))
                        plt.plot(astar_path[:,0], astar_path[:,1])

                    if astar_energy > 9999999999999999:
                        print ("AStar path is inf. On to next hotspot")
                        break

                
                total_ref_energy, total_ref_time, real_ref_cost, ref_ig = follow_path_waypoints_v2(
                                                                            full_ref_path,
                                                                            env, desired_speed, sample_sec, *[False])
                if len(full_astar_path) > 0:
                    total_astar_energy, total_astar_time, real_astar_cost, astar_ig = follow_path_waypoints_v2(
                                                                                full_astar_path,
                                                                                env, desired_speed, sample_sec, *[False])


                all_ref_energy_costs.append(total_ref_energy)
                all_ref_time_costs.append(total_ref_time)
                all_astar_energy_costs.append(total_astar_energy)
                all_astar_time_costs.append(total_astar_time)

                print ("Traveling from HTSP{0} to HTSP{1} at TIME{2}".format(hspt1, hspt2, sample_hrs))
                print ("COMPARE ALL THREE RESULTS NOW")
                print ("total_astar_energy: ", total_astar_energy)
                print ("total_astar_time  : ", total_astar_time)
                print ("total_ref_energy  : ", total_ref_energy)
                print ("total_ref_time    : ", total_ref_time)


                ##Visualize currents at this step
                fig, ax1 = plt.subplots()
                ax1.plot(np_all_hotspots[:, 0], np_all_hotspots[:,1], 'o')
                ax1.text(h0[0], h0[1], "H0", fontsize='20')
                ax1.text(h1[0], h1[1], "H1", fontsize='20')
                ax1.text(h2[0], h2[1], "H2", fontsize='20')
                ax1.text(h3[0], h3[1], "H3", fontsize='20')
                ax1.text(h4[0], h4[1], "H4", fontsize='20')
                ax1.text(h5[0], h5[1], "H5", fontsize='20')
                ax1.text(h6[0], h6[1], "H6", fontsize='20')

                min_width = min(all_hotspot[hspt1][0], all_hotspot[hspt2][0]) - 100
                max_width = max(all_hotspot[hspt1][0], all_hotspot[hspt2][0]) + 100
                min_height = min(all_hotspot[hspt1][1], all_hotspot[hspt2][1]) - 100
                max_height = max(all_hotspot[hspt1][1], all_hotspot[hspt2][1]) + 100

                width = abs(max_width - min_width)
                height = abs(max_height - min_height)
                xwidth = [min_width, max_width]
                yheight = [min_height, max_height]

                ufunc, vfunc, uvfunc = get_multi_current_block(place_bbox, currents_path1, currents_path2)
                d_area, dfunc = get_depth_block(place_bbox, topo_path)
                depths = grid_data(dfunc, xwidth, yheight, 100, [], [])
                depths = np.ones((depths.shape[0], depths.shape[1]))*0
                depths_flat = depths.flatten()
                # time_range = np.arange(0, 72, 1)
                time_range = np.arange(0, 30, 1)

                w,h = np.meshgrid(np.linspace(xwidth[0], xwidth[1], depths.shape[1], endpoint=True),
                                  np.linspace(yheight[0], yheight[1], depths.shape[0], endpoint=True))    

                u = grid_data(ufunc, xwidth, yheight, 500, depths, [sample_hrs])
                v = grid_data(vfunc, xwidth, yheight, 500, depths, [sample_hrs])
                uv = grid_data(uvfunc, xwidth, yheight, 500, depths, [sample_hrs])

                vmin = np.min(uv)
                vmax = np.max(uv)

                ax1.set_title('Nominal vs Astar from Hotspot{0} to Hotspot{1} at T={2} Hrs'.format(hspt1, hspt2, sample_hrs))
                im1 = ax1.quiver(w,h, u, v, uv)
                ax1.set_xlim(min_width, max_width)
                ax1.set_ylim(min_height, max_height)
                fig.colorbar(im1)

                ##plot ref and astar pattern
                ax1.plot(waypoints[:,0], waypoints[:,1])
                ax1.plot(full_astar_path[:,0], full_astar_path[:,1])

                plt.savefig('./san_nicolas_costs/rrt_astar_h{0}_to_h{1}_time{2}.png'.format(hspt1, hspt2, sample_hrs))
                # plt.clf()
                plt.close()


            print ("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            ###save these to a csv file
            results_txt_file = open('./san_nicolas_costs/rrt_paths_astar_ref.txt', "a")
            results_txt_file.write("\nREF HOTSPOT{0} TO HOTSPOT{1} AT TIME_HRS{2}\n".format(hspt1,hspt2,time_hrs[sample_hrs]))
            results_txt_file.write("\nEnergy: {0}".format(all_ref_energy_costs))
            results_txt_file.write("\nTime: {0}\n".format(all_ref_time_costs))
            results_txt_file.write(str(ref_path))

            results_txt_file.write("\nASTAR HOTSPOT{0} TO HOTSPOT{1} AT TIME_HRS{2}\n".format(hspt1,hspt2,time_hrs[sample_hrs]))
            results_txt_file.write("\nEnergy: {0}".format(all_astar_energy_costs))
            results_txt_file.write("\nTime: {0}\n".format(all_astar_time_costs))
            results_txt_file.write(str(astar_path))
            results_txt_file.close()

            title_row = ["Hspt{0}_to_Hspt{1}".format(hspt1, hspt2)] + time_hrs.tolist()
            ref_energy_row = ["REF_Energy_cost"] + all_ref_energy_costs
            ref_time_row = ["REF_Time_sec_cost"] + all_ref_time_costs
            astar_energy_row = ["ASTAR_Energy_cost"] + all_astar_energy_costs
            astar_time_row = ["ASTAR_Time_sec_cost"] + all_astar_time_costs

            with open('./san_nicolas_costs/rrt_ref_surface1_hspt{0}_energy_time_costs.csv'.format(hspt1), 'a') as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow(title_row)
                csvwriter.writerow(ref_energy_row)
                csvwriter.writerow(ref_time_row)


            with open('./san_nicolas_costs/rrt_astar_surface1_hspt{0}_energy_time_costs.csv'.format(hspt1), 'a') as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow(title_row)
                csvwriter.writerow(astar_energy_row)
                csvwriter.writerow(astar_time_row)






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

    # desired_speed = 1.5
    desired_speed = 2.5722

    #get waypoints
    np.random.seed(214323)

    h0 = np.array([0,0,0])
    h1 = np.array([ 2413.17937189, -1454.27348944,     0.        ] )
    h2 = np.array([-4955.77214741, -1807.45178534,     0.        ] )
    h3 = np.array([ 2431.56786602,  -895.66810652,     0.        ] )
    h4 = np.array([-2301.39176315,  3674.90015933,     0.        ] )
    h5 = np.array([-1590.5154935 ,   22.5489957  ,     0.        ] )
    h6 = np.array([  104.16639771, -4009.83609744,     0.        ] )
    all_hotspot = [h0, h1, h2, h3, h4, h5, h6]

    # visualize_multi_current(place_bbox, currents_path1, currents_path2, topo_path, h1, h3)


    # calc_nom_soln()
    # cal_ref_astar()

    # cost_to_waypoint_v2(h0, np.array([100, 100, 0]), 0.0, env, \
    #                     desired_speed, *[True])
    # cost_to_waypoint_v2(h0, np.array([100, -100, 0]), 0.0, env, \
    #                     desired_speed, *[True])
    # cost_to_waypoint_v2(h0, np.array([-100, 100, 0]), 0.0, env, \
    #                     desired_speed, *[True])
    # cost_to_waypoint_v2(h0, np.array([-100, -100, 0]), 0.0, env, \
    #                     desired_speed, *[True])

    # cost_to_waypoint_v2(h0, np.array([0, -100, 0]), 0.0, env, \
    #                     desired_speed, *[True])
    # cost_to_waypoint_v2(h0, np.array([-100, 0, 0]), 0.0, env, \
    #                     desired_speed, *[True])
    # cost_to_waypoint_v2(h0, np.array([0, 100, 0]), 0.0, env, \
    #                     desired_speed, *[True])
    # cost_to_waypoint_v2(h0, np.array([100, 0, 0]), 0.0, env, \
    #                     desired_speed, *[True])

    # htest = Point(500, 500, 0) 
    # htest = np.array([500, 500,0])
    # htest = np.array([500, -500,0])
    # htest = np.array([-500, 500,0])
    htest = np.array([-500, -500,0])
    astar_path = RT.step_to_point(h0, htest, 0.0)

    optx = np.linspace(h0[0], htest[0], 10).astype(int).reshape(-1,1)
    opty = np.linspace(h0[1], htest[1], 10).astype(int).reshape(-1,1)
    optz = np.zeros((len(optx),1))
    waypoints = np.hstack((optx, opty, optz))


    for w in range(len(waypoints)-1):
        print ("w {0} / {1} waypoints: ".format(w, len(waypoints)))
        ##get the amount of energy you need to beat for this part of the path
        all_w = np.vstack([waypoints[w], waypoints[w+1]])
        ref_energy, ref_time_sec, ref_est_cost, ref_path = follow_path_waypoints_v2(all_w, 
                                                        env, desired_speed, sample_sec, *[False])


    astar_energy, astar_time, meh, meh2 = follow_path_waypoints_v2( astar_path, 
                                                        env, desired_speed, sample_sec, *[False])





    # step_size = 100
    # goal = np.array([500, 500, 0])
    # max_dist = np.linalg.norm([h0 - goal])
    # h0_P = Point(h0[0], h0[1], h0[2]) 
    # ball = RT.calculate_ball(h0_P, step_size)
    # ball_z = np.zeros((ball.shape[0])).reshape(-1,1)
    # ball = np.hstack((ball, ball_z))



    # fig, ax1 = plt.subplots()
    # min_width = -50
    # max_width = 550
    # min_height = 0
    # max_height = 550
    # sample_hrs = 0

    # # min_width = min(all_hotspot[hspt1][0], all_hotspot[hspt2][0]) - 100
    # # max_width = max(all_hotspot[hspt1][0], all_hotspot[hspt2][0]) + 100
    # # min_height = min(all_hotspot[hspt1][1], all_hotspot[hspt2][1]) - 100
    # # max_height = max(all_hotspot[hspt1][1], all_hotspot[hspt2][1]) + 100

    # width = abs(max_width - min_width)
    # height = abs(max_height - min_height)
    # xwidth = [min_width, max_width]
    # yheight = [min_height, max_height]

    # ufunc, vfunc, uvfunc = get_multi_current_block(place_bbox, currents_path1, currents_path2)
    # d_area, dfunc = get_depth_block(place_bbox, topo_path)
    # depths = grid_data(dfunc, xwidth, yheight, 100, [], [])
    # depths = np.ones((depths.shape[0], depths.shape[1]))*0
    # depths_flat = depths.flatten()

    # w,h = np.meshgrid(np.linspace(xwidth[0], xwidth[1], depths.shape[1], endpoint=True),
    #                   np.linspace(yheight[0], yheight[1], depths.shape[0], endpoint=True))    

    # u = grid_data(ufunc, xwidth, yheight, 500, depths, [sample_hrs])
    # v = grid_data(vfunc, xwidth, yheight, 500, depths, [sample_hrs])
    # uv = grid_data(uvfunc, xwidth, yheight, 500, depths, [sample_hrs])

    # vmin = np.min(uv)
    # vmax = np.max(uv)

    # im1 = ax1.quiver(w,h, u, v, uv)



    # ax1.plot(h0[0], h0[1], 'o')
    # ax1.plot(ball[:,0], ball[:,1], 'o')
    # ax1.plot(goal[0], goal[1], 'o')

    # me = h0

    # while np.linalg.norm([me  - goal])>10:
    #     h0_P = Point(me[0], me[1], me[2]) 
    #     ball = RT.calculate_ball(h0_P, step_size)
    #     ball_z = np.zeros((ball.shape[0])).reshape(-1,1)
    #     ball = np.hstack((ball, ball_z))

    #     print ("ball: ", ball)
    #     ball_scores = []
    #     max_score = -999
    #     for b in ball:
    #         astar_score = currents_score(h0, b, 0.0, env, max_dist, 0.2)
    #         print ("astar_score: ", astar_score)
    #         ball_scores.append(astar_score)
    #         if astar_score > max_score:
    #             max_score = astar_score
    #             max_pt = b

    #         ax1.text(b[0], b[1], str(astar_score), fontsize=10)

    #     ax1.text(max_pt[0]+15, max_pt[1]+15, 'MIN SCORE')
    #     me = max_pt
    #     plt.pause(1)

    # plt.show()



    # astar_path = RT.step_to_point(h0, np.array([500, 500, 0]), 0.0)
    # follow_path_waypoints_v2(full_astar_path,env, desired_speed, sample_sec, *[False])


'''

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

            plt.plot(nom_path[:,0], nom_path[:,1])

            print ("NOM_ENERGY: ", nom_energy)
            print ("NOM TIME  : ", nom_time_sec)
            print ("NOM EST   : ", nom_est_cost)

            all_energy_costs[hspt1, hspt2] = nom_energy
            all_time_costs[hspt1, hspt2] = nom_time_sec


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
                        else:
                            print ("ref is better, moving onto next point")
                            print ("REF ENERGY: ", ref_energy)
                            print ("REF TIME  : ", ref_time_sec)
                            print ("astar energy: ", astar_energy)
                            print ("astar time  : ", astar_time)

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
'''