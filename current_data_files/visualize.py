import pickle

import rospkg
import numpy as np
from scipy.io import netcdf
from scipy.interpolate import griddata
import matplotlib.pyplot as plt

from trash_utils.finder_utils import get_current_block, get_depth_block, lat_lon_to_xy

'''
Visualization methods for current and depth data
'''

def visualize_currents(bbox, current_file):
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


def main():
    rospack = rospkg.RosPack()
    trash_finder_path = rospack.get_path("trash_finder")
    data_path = rospack.get_path("data_files")

    config = pickle.load(open(trash_finder_path + '/config/demo_configs.p', 'rb'))

    ##hydrodynamic current
    current_path = data_path+'/ca_subCA_das_2020010615.nc'
    current_data = netcdf.NetCDFFile(current_path)
    depth = current_data.variables['depth'][:].copy()
    current_data.close()

    ##bathymetry model
    topo = config['topo_file']
    topo_path = data_path + '/' + topo

    all_locations = pickle.load( open(trash_finder_path+"/config/locations.p", "rb"))
    place_bbox = all_locations["mission_bay_flatter_bbox"]

    #visualize_currents(place_bbox, current_path)
    #visualize_depths (place_bbox, topo_path)

main()