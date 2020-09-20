# from math import atan2, sin, cos, acos
# from copy import deepcopy
# from datetime import datetime
# import csv
import pickle

import numpy as np
import shapely.geometry as sg
from shapely.ops import nearest_points
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from trash_utils.Env import Env
# from trash_utils.UUV import UUV
# from trash_utils.finder_utils import (grid_data, fix_waypoint_edges)
from trash_utils.trash_lib import ( init_hotspots, 
                                    visualize_trash_flow,
                                    update_trash_pos,
                                    visualize_trash_step)
# from trash_utils.cc_utils import calc_mowing_lawn, search_for_trash
# from trash_utils.fourD_utils import (cost_to_waypoint_v1, 
                                     # follow_path_waypoints, 
                                     # calculate_nominal_path_short,
                                     # follow_path_order)

import pdb


def init_hotspot_dict(env):
    ## Create hotspots of trash and (optinal) visualize
    np.random.seed(29435)
    trash_x_centers = np.random.randint(env.xbound[0], env.xbound[1], 6).reshape(-1,1)
    trash_y_centers = np.random.randint(env.ybound[0], env.ybound[1], 6).reshape(-1,1)
    ##Use this to auto generate gaussian trash distributions:
    trash_sigma = [[], []]               
    hotspot_dict = init_hotspots(trash_x_centers, trash_y_centers,
                                 trash_sigma, 20, env.dfunc)
    for xx in range(1000):
        hotspot_dict = update_trash_pos(hotspot_dict, 0, env, True)

    return hotspot_dict


def trash_positions(hotspot_dict, env, limits):
    time_hrs = np.arange(0, 72, 0.5)
    # time_hrs = np.arange(0, 5, 0.5)

    time_slots_t = []
    time_slots_diffsec = []

    for t in range(len(time_hrs)):
        ##Update the trash positions up to this time
        if t>0:
            print ("Updating trash positions for t: {0}".format(t))
            for diff_sec in range(int(time_hrs[t-1]*3600), int(time_hrs[t]*3600), 10):
                diff_hrs = diff_sec/3600
                time_slots_t.append(t)
                time_slots_diffsec.append(diff_hrs)
                hotspot_dict = update_trash_pos(hotspot_dict, diff_hrs, env, True)

        if t%2 == 0:
            visualize_snapshot_pos(hotspot_dict, t, limits)
            # pdb.set_trace()

    # pdb.set_trace()
    return hotspot_dict


def visualize_all_pos(hotspot_dict):
    '''
    For both 2D and 3D

    '''
    for idx in hotspot_dict.keys():
        trash_pos = np.array(hotspot_dict[idx])

        ##Visualize 2D
        fig = plt.figure()
        plt.plot(trash_pos[:,:,0], trash_pos[:,:,1], '-')
        # plt.plot(trash_pos[:,:,0], trash_pos[:,:,1], 'o')
        plt.title('Hotspot {0} from Time 0 to 72hrs'.format(idx))
        # plt.show()
        plt.savefig('./hotspot{0}_2d_all_lines.png'.format(idx))


        ##Visualize 3D
        fig = plt.figure()
        ax1 = fig.gca(projection='3d')
        for t in range(trash_pos.shape[1]):
            x = trash_pos[:,t,0]
            y = trash_pos[:,t,1]
            z = trash_pos[:,t,2]
            # ax1.plot(x, y, z, 'o')
            ax1.plot(x, y, z, '-')
        plt.title('Hotspot {0} from Time 0 to 72hrs'.format(idx))
        # plt.show()
        plt.savefig('./hotspot{0}_3d_all_lines'.format(idx))


def visualize_snapshot_pos(hotspot_dict, savehr, limits):
    '''


    '''

    for idx in hotspot_dict.keys():
        trash_pos = np.array(hotspot_dict[idx])

        ##Visualize 2D
        fig = plt.figure()
        # plt.plot(trash_pos[0:-1:spacing, : ,0], trash_pos[0:-1:spacing, :, 1])
        plt.plot(trash_pos[-1,:,0], trash_pos[-1,:,1], 'o')
        plt.title('Hotspot {0} at Hour: {1}'.format(idx, savehr/2))
        plt.xlim([limits[idx][0], limits[idx][1]])
        plt.ylim([limits[idx][2], limits[idx][3]])
        # plt.show()
        plt.savefig('./hotspot{0}/hotspot{0}_2d_hr{1}.png'.format(idx, savehr/2))


        ##Visualize 3D
        fig = plt.figure()
        ax1 = fig.gca(projection='3d')
        # ax1.plot(trash_pos[:,:,0], trash_pos[:,:,1], trash_pos[:,:,2])
        ax1.plot(trash_pos[-1, :, 0], trash_pos[-1, :, 1], trash_pos[-1, :, 2], 'o')
        plt.title('Hotspot {0} at Hour: {1}'.format(idx, savehr/2))
        ax1.set_xlim([limits[idx][0], limits[idx][1]])
        ax1.set_ylim([limits[idx][2], limits[idx][3]])
        ax1.set_zlim([limits[idx][4], limits[idx][5]])
        # plt.show()
        plt.savefig('./hotspot{0}/hotspot{0}_3d_hr{1}.png'.format(idx, savehr/2))



def read_hotspot_dict():
    '''
    Use this to read in the dictionary

    '''
    with open('hotspot_trash_pos_all.pickle', 'rb') as f:
        hotspot_dict = pickle.load(f)

    return hotspot_dict



if __name__ == '__main__':
    
    env = Env(1, 1) ##This contains (ufunc, vfunc, width, height, u_boost, v_boost)
    limits = [[2150.1262564264507, 2570.4840844206387, -692.90694603696193, -306.98315236535564, 813.99999999999989, 1015.0000000000001], [-5408.9012705449541, -5088.6704538171916, -2636.9068487677114, -2272.1163527808976, 798.99999999999989, 999.99387841783243], [2177.8528354874702, 2542.9390838912091, -2979.5984074559933, -2617.9097735746514, 820.31737161960916, 1023.5888348279548], [5277.5233628111255, 5595.746792771004, -2107.6156224916253, -1709.6081274141161, 819.99999999999989, 1020.0000000000001], [-2774.5624693017303, -2416.6364728072476, 2491.1959806627801, 2871.6014413164071, 820.0, 1021.9515118288564], [-2081.3046519556292, -1695.3010310413663, -1083.6644030061584, -747.87525739361229, 809.99999999999989, 1011.0000000000002]]


##Create the data
    hotspot_dict = init_hotspot_dict(env)
    hotspot_dict = trash_positions(hotspot_dict, env, limits)

    # with open('hotspot_trash_pos_all.pickle', 'wb') as f:
    #     pickle.dump(hotspot_dict, f, pickle.HIGHEST_PROTOCOL)

##Or load the data
    # hotspot_dict = read_hotspot_dict()

    # limits = []
    # for idx in hotspot_dict.keys():
    #     trash_pos = np.array(hotspot_dict[idx])
    #     xlim_min = min(trash_pos[-1,:,0]) - 100
    #     xlim_max = max(trash_pos[-1,:,0]) + 100
    #     ylim_min = min(trash_pos[-1,:,1]) - 100
    #     ylim_max = max(trash_pos[-1,:,1]) + 100
    #     zlim_min = min(trash_pos[-1,:,2]) - 100
    #     zlim_max = max(trash_pos[-1,:,2]) + 100

    #     limits.append([xlim_min, xlim_max, ylim_min, ylim_max, zlim_min, zlim_max])

    # print (limits)

##Visualize data
    # visualize_all_pos(hotspot_dict)
    # visualize_snapshot_pos(hotspot_dict)