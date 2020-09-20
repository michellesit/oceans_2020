import matplotlib.pyplot as plt
import numpy as np

from math import atan2, sin, cos, acos
import math
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
# from trash_utils.cc_utils import calc_mowing_lawn, search_for_trash
# from trash_utils.fourD_utils import (cost_to_waypoint_v2, 
                                     # follow_path_waypoints_v2, 
                                     # calculate_nominal_path_short,
                                     # follow_path_order,
                                     # currents_score)
from trash_utils.visualize import (visualize_multi_current)

from trash_utils.finder_utils import get_current_block, get_depth_block, lat_lon_to_xy, grid_data, get_multi_current_block

# from hs4dpp_main_utils import calc_heuristic_denom, calc_all_headings
# from hs4dpp_path_utils import calc_ball_cost, find_optimal_path_nrmpc

import pdb

import pickle
from tqdm import tqdm

import rospkg
from scipy.io import netcdf
from trash_utils.haversine_dist import haversine



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

# desired_speed = 1.5
desired_speed = 2.5722

#get waypoints
np.random.seed(214323)

# visualize_multi_current(place_bbox, currents_path1, currents_path2, topo_path, h1, h3)


fig, ax1 = plt.subplots()

place_bbox = np.array([[33.517, -119.900],
                        [33.517, -119.786],
                        [33.428, -119.786],
                        [33.428, -119.900]])

h0 = np.array([0,0,0])
h1 = np.array([ 2413.17937189, -1454.27348944,     0.        ] )
h2 = np.array([-4955.77214741, -1807.45178534,     0.        ] )
h3 = np.array([ 2431.56786602,  -895.66810652,     0.        ] )
h4 = np.array([-2301.39176315,  3674.90015933,     0.        ] )
h5 = np.array([-1590.5154935 ,   22.5489957  ,     0.        ] )
h6 = np.array([  104.16639771, -4009.83609744,     0.        ] )
all_hotspot = [h0, h1, h2, h3, h4, h5, h6]
np_all_hotspots = np.array(all_hotspot)



##move the trash around for all these hotspots
    ## Create hotspots of trash and (optinal) visualize
    # trash_x_centers = np.array([-250.98494701, -504.8406451, \
                                # -132, 345, 876, 423]).reshape(-1,1)
    # trash_y_centers = np.array([-508.96243035, -877.89326774, \
                                # -687, 354, 120, 348]).reshape(-1,1)

    # np.random.seed(29435)
    # trash_x_centers = np.random.randint(env.xbound[0], env.xbound[1], 6).reshape(-1,1)
    # trash_y_centers = np.random.randint(env.ybound[0], env.ybound[1], 6).reshape(-1,1)
    # ##Use this to auto generate gaussian trash distributions:
    # trash_sigma = [[], []]               
    # hotspot_dict = init_hotspots(trash_x_centers, trash_y_centers,
    #                              trash_sigma, 20, env.dfunc)
    # for xx in range(1000):
    #     hotspot_dict = update_trash_pos(hotspot_dict, 0, env)

trash_x_centers = np_all_hotspots[:,0].reshape(-1,1)
trash_y_centers = np_all_hotspots[:,1].reshape(-1,1)
trash_sigma = [[], []]

# time_hrs = np.arange(0, 72, 1)
time_hrs = np.arange(0, 50, 1)

hotspot_dict = init_hotspots(trash_x_centers, trash_y_centers, trash_sigma, 20, env.dfunc)
for xx in range(1000):
    hotspot_dict = update_trash_pos(hotspot_dict, 0, env)


for t in range(len(time_hrs)):
    ##Update the trash positions up to this time
    if t>0:
        print ("Updating trash positions for t: {0}".format(t))
        for diff_sec in range(int(time_hrs[t-1]*3600), int(time_hrs[t]*3600), 10):
            diff_hrs = diff_sec/3600.0
            hotspot_dict = update_trash_pos(hotspot_dict, diff_hrs, env)


for hspt in hotspot_dict.keys():
    ## Get convex hull of each hotspot plus some buffer
    convexhull_a = sg.MultiPoint(hotspot_dict[hspt][-1][:, 0:2]).convex_hull
    buffer_a = convexhull_a.buffer(5)
    ax, ay = buffer_a.exterior.coords.xy
    print ("area: ", hspt, buffer_a.area)

    plt.plot(ax, ay, '-')

plt.show()

pdb.set_trace()

## Get the convex hull of the corresponding hotspot plus some buffer
# convexhull_b = sg.MultiPoint(hotspot_dict[hspt2][-1][:, 0:2]).convex_hull
# buffer_b = convexhull_b.buffer(5)
# bx, by = buffer_b.exterior.coords.xy

##Calculate the closest points between the hotspots
# pt1, pt2 = nearest_points(buffer_a, buffer_b)



# min_width = min(np_all_hotspots[:,0]) - 100
# max_width = max(np_all_hotspots[:,0]) + 100
# min_height = min(np_all_hotspots[:,1]) - 100
# max_height = max(np_all_hotspots[:,1]) + 100

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

# ax1.plot([h0[0], h3[0]], [h0[1], h3[1]], '-')
# ax1.plot([h3[0], h1[0]], [h3[1], h1[1]], '-')
# ax1.plot([h1[0], h6[0]], [h1[1], h6[1]], '-')
# ax1.plot([h0[0], h5[0]], [h0[1], h5[1]], '-')
# ax1.plot([h5[0], h3[0]], [h5[1], h3[1]], '-')
# ax1.plot([h1[0], h5[0]], [h1[1], h5[1]], '-')
# ax1.plot([h5[0], h2[0]], [h5[1], h2[1]], '-')
# ax1.plot([h2[0], h6[0]], [h2[1], h6[1]], '-')
# ax1.plot([h0[0], h1[0]], [h0[1], h1[1]], '-')
# ax1.plot([h1[0], h3[0]], [h1[1], h3[1]], '-')
# ax1.plot([h3[0], h4[0]], [h3[1], h4[1]], '-')
# ax1.plot([h4[0], h2[0]], [h4[1], h2[1]], '-')


# ax1.plot(np_all_hotspots[:,0], np_all_hotspots[:,1], 'o')
# ax1.text(h0[0], h0[1], "H0", fontsize='20')
# ax1.text(h1[0], h1[1], "H1", fontsize='20')
# ax1.text(h2[0], h2[1], "H2", fontsize='20')
# ax1.text(h3[0], h3[1], "H3", fontsize='20')
# ax1.text(h4[0], h4[1], "H4", fontsize='20')
# ax1.text(h5[0], h5[1], "H5", fontsize='20')
# ax1.text(h6[0], h6[1], "H6", fontsize='20')



# ax1.text(h0[0], h0[1], "H0", fontsize='20')
# ax1.text(h1[0], h1[1], "H1", fontsize='20')
# ax1.text(h2[0], h2[1], "H2", fontsize='20')
# ax1.text(h3[0], h3[1], "H3", fontsize='20')
# ax1.text(h4[0], h4[1], "H4", fontsize='20')
# ax1.text(h5[0], h5[1], "H5", fontsize='20')
# ax1.text(h6[0], h6[1], "H6", fontsize='20')
# plt.show()