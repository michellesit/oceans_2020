import pickle

import rospkg
import numpy as np
from scipy.io import netcdf
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from trash_utils.haversine_dist import haversine
from trash_utils.finder_utils import get_current_block, get_depth_block, lat_lon_to_xy, grid_data, get_multi_current_block

import pdb

'''
Visualization methods for current and depth data
'''

def visualize_currents(bbox, current_file, depth):
    '''
    Visualize the currents at each depth

    Input:
        bbox (np.ndarray): 4x2 array with lon/lat float coordinates for the area of interest
            Coordinates should be top left, top right, bottom right, bottom left
            Each bbox corner should be in [lon, lat] order
        current_file (string): path to netcdf file containing the hydrodynamic data

    '''
    min_lat = min(bbox[:, 0])
    max_lat = max(bbox[:, 0])
    min_lon = min(bbox[:, 1])
    max_lon = max(bbox[:, 1])    
    
    u_area, v_area, uv_area, interp_u, interp_v, interp_uv = get_current_block(bbox, current_file)

    ##make into vector
    uv_area = np.hypot(u_area, v_area)

    ##To standardize the colorbar, find the max and min values
    ##Set the colorbar limits for each image
    vmin = np.min(uv_area)
    vmax = np.max(uv_area)
    
    ##Visualize
    fig = plt.figure(1)
    rows = 3
    cols = 5
    position = range(1, depth.shape[0]+1)
    fig.suptitle('Current vector field')
    for d in range(depth.shape[0]):
        u_slice = u_area[d,:,:]
        v_slice = v_area[d,:,:]
        X,Y = np.meshgrid(np.linspace(min_lon, max_lon, v_slice.shape[1], endpoint=True),
                          np.linspace(min_lat, max_lat, u_slice.shape[0], endpoint=True))
        
        ax1 = fig.add_subplot(rows, cols, position[d])
        ax1.set_title("(depth={0})".format(depth[d]))
        ax1.set_xlabel('latitude')
        ax1.set_ylabel('longitude')

        ax1.scatter(X,Y, color='b', s=15)
        im1 = ax1.quiver(X,Y, u_slice, v_slice, uv_area[d,:,:])
        ax1.set_xlim([X[0][0]-0.01, X[0][-1]+0.01])
        ax1.set_ylim([Y[0][0]-0.01, Y[0][-1]+0.03])
        ax1.axis('equal')
        
        fig.colorbar(im1)
        im1.set_clim(vmin, vmax)

    plt.show()


def visualize_depths(bbox, topo_file):
    '''
    Visualize depth map

    Input:
        bbox (np.ndarray): 4x2 array with lon/lat float coordinates for the area of interest
            Coordinates should be top left, top right, bottom right, bottom left
            Each bbox corner should be in [lon, lat] order
        topo_file (string): path to netcdf file containing the bathymetry data

    '''

    min_lat = min(bbox[:, 0])
    max_lat = max(bbox[:, 0])
    min_lon = min(bbox[:, 1])
    max_lon = max(bbox[:, 1]) 

    depths_area, depths_func = get_depth_block(bbox, topo_file)

    lon,lat = np.meshgrid(np.linspace(min_lon, max_lon, depths_area.shape[1], endpoint=True),
                          np.linspace(min_lat, max_lat, depths_area.shape[0], endpoint=True))

    x, y = lat_lon_to_xy(bbox, lat, lon)
    grid = zip(x.flatten(), y.flatten())
    z = np.array(depths_func(grid)).reshape((depths_area.shape))

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111)
    im = plt.imshow(np.flipud(z))
    fig.colorbar(im)
    plt.show()


def visualize_area(bbox, currents_path, topo_path, fig_title='Current vector field'):
    '''
    visualizes all pieces

    '''
    min_lat = min(bbox[:, 0])
    max_lat = max(bbox[:, 0])
    min_lon = min(bbox[:, 1])
    max_lon = max(bbox[:, 1]) 

    width = haversine(bbox[0,0], bbox[0,1], bbox[1,0], bbox[1,1]) * 1000
    height = haversine(bbox[1,0], bbox[1,1], bbox[2,0], bbox[2,1]) * 1000

    xwidth = [-width/2, width/2]
    yheight = [-height/2, height/2]

    u_area, v_area, uv_area, ufunc, vfunc, uvfunc = get_current_block(bbox, currents_path)
    d_area, dfunc = get_depth_block(bbox, topo_path)

    depths = grid_data(dfunc, xwidth, yheight, 10, [])
    depths += 1
    depths_flat = depths.flatten()

    u = grid_data(ufunc, xwidth, yheight, 10, depths)
    v = grid_data(vfunc, xwidth, yheight, 10, depths)
    uv = grid_data(uvfunc, xwidth, yheight, 10, depths)
    # angles = np.arctan2(u, v)

    vmin = np.min(uv)
    vmax = np.max(uv)
    
    ##Visualize 2D
    fig = plt.figure(1)
    fig.suptitle(fig_title)

    w,h = np.meshgrid(np.linspace(-width/2, width/2, depths.shape[1], endpoint=True),
                      np.linspace(-height/2, height/2, depths.shape[0], endpoint=True))

    ax1 = fig.add_subplot(1, 1, 1)
    ax1.set_xlabel('width')
    ax1.set_ylabel('height')

    ax1.scatter(w,h, color='b', s=15)
    im1 = ax1.quiver(w,h, u, v, uv)
    fig.colorbar(im1)
    im1.set_clim(vmin, vmax)
    # plt.show()

    #############

    # ##3D
    # fig = plt.figure(2)
    # fig.suptitle('Current vector field')

    # z_space = np.where(file_depths<abs(min(depths_flat)))
    # z_param = file_depths[0:z_space[0][-1]+1]

    # w,h, z = np.meshgrid(np.linspace(-width/2, width/2, depths.shape[1], endpoint=True),
    #                      np.linspace(-height/2, height/2, depths.shape[0], endpoint=True),
    #                      z_param)

    # # ax1 = fig.add_subplot(1, 1, 1, projection='3d')
    # ax1 = Axes3D(fig)
    # ax1.set_xlabel('width')
    # ax1.set_ylabel('height')

    # ax1.scatter(w,h, color='b', s=15)
    # im1 = ax1.quiver(w,h,z, u, v, uv)
    # # fig.colorbar(im1)
    # # im1.set_clim(vmin, vmax)
    # plt.show()    


def visualize_multi_current(bbox, currents1_path, currents2_path, topo_path, ht1, ht2, t_step=0):
    '''
    visualizes all pieces

    '''
    h0 = np.array([0,0,0])
    h1 = np.array([ 2413.17937189, -1454.27348944,     0.        ] )
    h2 = np.array([-4955.77214741, -1807.45178534,     0.        ] )
    h3 = np.array([ 2431.56786602,  -895.66810652,     0.        ] )
    h4 = np.array([-2301.39176315,  3674.90015933,     0.        ] )
    h5 = np.array([-1590.5154935 ,   22.5489957  ,     0.        ] )
    h6 = np.array([  104.16639771, -4009.83609744,     0.        ] )
    all_hotspot = [h0, h1, h2, h3, h4, h5, h6]

    ##Visualize currents at this step
    # fig = plt.figure()
    fig, ax1 = plt.subplots()
    ahhh = np.array(all_hotspot)
    # plt.plot(ahhh[:,0], ahhh[:,1], 'o', markersize=20)
    ax1.plot(all_hotspot[ht1][0], all_hotspot[ht1][1], 'o')
    ax1.plot(all_hotspot[ht2][0], all_hotspot[ht2][1], 'o')


    ax1.text(h0[0], h0[1], "H0", fontsize='20')
    ax1.text(h1[0], h1[1], "H1", fontsize='20')
    ax1.text(h2[0], h2[1], "H2", fontsize='20')
    ax1.text(h3[0], h3[1], "H3", fontsize='20')
    ax1.text(h4[0], h4[1], "H4", fontsize='20')
    ax1.text(h5[0], h5[1], "H5", fontsize='20')
    ax1.text(h6[0], h6[1], "H6", fontsize='20')


    min_lat = min(bbox[:, 0])
    max_lat = max(bbox[:, 0])
    min_lon = min(bbox[:, 1])
    max_lon = max(bbox[:, 1]) 

    width = haversine(bbox[0,0], bbox[0,1], bbox[1,0], bbox[1,1]) * 1000
    height = haversine(bbox[1,0], bbox[1,1], bbox[2,0], bbox[2,1]) * 1000

    print ("width: ", width/1000)
    print ("height: ", height/1000)

    xwidth = [-width/2, width/2]
    yheight = [-height/2, height/2]





    min_width = min(all_hotspot[ht1][0], all_hotspot[ht2][0]) - 100
    max_width = max(all_hotspot[ht1][0], all_hotspot[ht2][0]) + 100
    min_height = min(all_hotspot[ht1][1], all_hotspot[ht2][1]) - 100
    max_height = max(all_hotspot[ht1][1], all_hotspot[ht2][1]) + 100

    width = abs(max_width - min_width)
    height = abs(max_height - min_height)

    xwidth = [min_width, max_width]
    yheight = [min_height, max_height]

    # pdb.set_trace()

    ufunc, vfunc, uvfunc = get_multi_current_block(bbox, currents1_path, currents2_path)
    d_area, dfunc = get_depth_block(bbox, topo_path)

    depths = grid_data(dfunc, xwidth, yheight, 100, [], [])
    # depths += 1
    # depths_flat = depths.flatten()

    depths = np.ones((depths.shape[0], depths.shape[1]))*0
    depths_flat = depths.flatten()
    # pdb.set_trace()
    # time_range = range(0, 145, 3)
    # time_range = range(25, 48, 3)
    # time_range = np.arange(0, 72, 0.5)
    time_range = np.arange(0, 72, 1)

    ##init figure outside
    ##8 figures for each day
    ##6 days total

    ##Visualize 2D
    # w,h = np.meshgrid(np.linspace(-width/2, width/2, depths.shape[1], endpoint=True),
    #                   np.linspace(-height/2, height/2, depths.shape[0], endpoint=True))    

    w,h = np.meshgrid(np.linspace(xwidth[0], xwidth[1], depths.shape[1], endpoint=True),
                  np.linspace(yheight[0], yheight[1], depths.shape[0], endpoint=True))    

    # fig = plt.figure(1)
    # fig.suptitle('Current vector field over time (day2)')
    rows = 2
    cols = 4

    # for t_step in range(len(time_range)):
    # t_step = 0
    # fig = plt.figure()
    # fig, ax1 = plt.subplots()
    u = grid_data(ufunc, xwidth, yheight, 500, depths, [time_range[t_step]])
    v = grid_data(vfunc, xwidth, yheight, 500, depths, [time_range[t_step]])
    uv = grid_data(uvfunc, xwidth, yheight, 500, depths, [time_range[t_step]])

    vmin = np.min(uv)
    vmax = np.max(uv)

    # pdb.set_trace()


    # ax1 = fig.add_subplot(1,1.1) 
    # ax1 = fig.add_subplot(111)       
    # ax1 = fig.add_subplot(rows, cols, t_step+1)
    # ax1.set_title("(time(hrs)={0})".format(time_range[t_step]))
    ax1.set_title('Nominal Path from Hotspot{0} to Hotspot{1} at T={2} Hrs'.format(ht1, ht2, t_step))
    ax1.set_xlabel('latitude')
    ax1.set_ylabel('longitude')

    # ax1.scatter(w,h, color='b', s=15)
    # ax1.scatter(w,h, color='b')
    im1 = ax1.quiver(w,h, u, v, uv)
    # ax1.plot(all_hotspot[ht1][0],all_hotspot[ht1][1], 'o', size=10)
    # ax1.text(all_hotspot[ht1][0],all_hotspot[ht1][1], "H{0}".format(ht1), fontsize='20')
    # ax1.plot(all_hotspot[ht2][0],all_hotspot[ht2][1], 'o', size=10)
    # ax1.text(all_hotspot[ht2][0],all_hotspot[ht2][1], 'H{0}'.format(ht2), fontsize='20')

    # fig.colorbar(im1)
    # im1.set_clim(vmin, vmax)

    # ax1.set_xlim([w[0][0]-0.01, w[0][-1]+0.01])
    # ax1.set_ylim([h[0][0]-0.01, h[0][-1]+0.03])

    ax1.set_xlim(min_width, max_width)
    ax1.set_ylim(min_height, max_height)
    # ax1.axis('equal')

    

    # fig.colorbar(im1, orientation='horizontal')
    fig.colorbar(im1)
    # im1.set_clim(0, 0.15)
    # plt.show()





def main():
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
    # place_bbox = all_locations["mission_bay_flatter_bbox"]
    # place_bbox = all_locations["santa_monica_basin"]
    # print (place_bbox)

    # pdb.set_trace()
    # place_bbox = np.array([[34.406, -122.485], [34.406, -117.245], [32.569, -117.245],[32.569, -122.485]])
    place_bbox = np.array([[33.517, -119.900],
                            [33.517, -119.786],
                            [33.428, -119.786],
                            [33.428, -119.900]])
    #     min_lat = -122.485
    # max_lat = -117.245
    # min_lon = 34.406
    # max_lon = 32.569
    # "santa_monica_basin" : np.array([[33.798, -118.924],
    #                                  [33.798, -118.805],
    #                                  [33.691, -118.805],
    #                                  [33.691, -118.924]]),

    # visualize_currents(place_bbox, current_path, depth)
    #visualize_depths (place_bbox, topo_path)

    # visualize_area(place_bbox, depth, current_path, topo_path)
    # visualize_multi_current(place_bbox, currents_path1, currents_path2, topo_path)

# main()
