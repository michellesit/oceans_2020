import pickle
import yaml

import rospkg
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import netcdf
from scipy.interpolate import RegularGridInterpolator

from trash_utils.haversine_dist import haversine

import pdb

'''
Utility functions to support the trash_finder/scripts files

'''

def xy_to_lat_lon(bbox, x,y):
    '''
    Converts x,y coordinates to lat/lon coordinates

    Input:
        bbox (np.ndarray)        : 4x2 array with lon/lat float coordinates for the area 
                                    of interest
                                   Coordinates should be top left, top right,
                                                         bottom right, bottom left
                                   Each bbox corner should be in [lon, lat] order
        x (float or np.ndarray)  : value(s) from coordinate system to convert to lat
        y (float or np.ndarray)  : value(s) from coordinate system to convert to lon

    Returns:
        lat (float or np.ndarray): corresponding lat value(s) to x coordinate(s)
        lon (float or np.ndarray): corresponding lon value(s) to y coordinate(s)

    '''
    width = haversine(bbox[0,0], bbox[0,1], bbox[1,0], bbox[1,1]) * 1000
    height = haversine(bbox[1,0], bbox[1,1], bbox[2,0], bbox[2,1]) * 1000

    ##assert the x and y values are valid
    ##depends on whether the values are zeroed or centered
    if not ((0 <= np.absolute(x).all() <= width/2)):
        raise Exception ("X VALUE(S) OUTSIDE BOUNDS. LIMIT +/-{0}".format(width/2))
    if not ((0 <= np.absolute(y).all() <= height/2)):
        raise Exception ("Y VALUE(S) OUTSIDE BOUNDS. LIMIT +/-{0}".format(height/2))

    min_lat = min(bbox[:, 0])
    max_lat = max(bbox[:, 0])
    min_lon = min(bbox[:, 1])
    max_lon = max(bbox[:, 1])

    lon = ((x + width/2)/width)*(max_lon - min_lon) + min_lon
    lat = ((y + height/2)/height)*(max_lat - min_lat) + min_lat

    return lat, lon


def lat_lon_to_xy(bbox, lat, lon):
    '''
    Converts lat/lon coordinates to x,y coordinates

    Input:
        bbox (np.ndarray)        : 4x2 array with lon/lat float coordinates for the area 
                                    of interest
                                   Coordinates should be top left, top right,
                                                         bottom right, bottom left
                                   Each bbox corner should be in [lon, lat] order
        lat (float or np.ndarray): lattitude value(s) to convert to y coordinate
        lon (float or np.ndarray): longitude value(s) to convert to x coordinate
    Return:
        x (float or np.ndarray)  : corresponding x value(s) to the lon coordinate
        y (float or np.ndarray)  : corresponding y value(s) to the lat coordinate

    '''

    ##assert that the values are valid within limits
    if not ((0 <= np.absolute(lat).all() <= 90)):
        raise Exception ("INVALID LAT INPUT(s): LIMIT -90 to 90")
    if not ((0 <= np.absolute(lon).all() <= 180)):
        raise Exception ("INVALID LON INPUT(s): LIMIT -180 to 180") 

    width = haversine(bbox[0,0], bbox[0,1], bbox[1,0], bbox[1,1]) * 1000
    height = haversine(bbox[1,0], bbox[1,1], bbox[2,0], bbox[2,1]) * 1000

    min_lat = min(bbox[:, 0])
    max_lat = max(bbox[:, 0])
    min_lon = min(bbox[:, 1])
    max_lon = max(bbox[:, 1])

    x = ((lon - min_lon)/(max_lon-min_lon) * width) - (width/2)
    y = ((lat - min_lat)/(max_lat-min_lat) * height) - (height/2)

    return x,y


def find_bound_idx(data, coord):
    '''
    Given a point and data array that may not have the specific point, 
    Finds the two values that bound the point. 
    Returns the two index values of those points
    
    This method is written to handle negative data and values
    
    Input:
        data (np.ndarray) : array containing all the data
        coord (list)      : Contains some data point that we need to find the bounds of
                            Could be either a single value (depth) 
                            OR a min and max value (currents)

    Returns:
        (np.ndarray): np array containing:
            min (int): idx value of the minimum bound
            max (int): idx value of the maximum bound
    
    '''
    ##make absolute value to handle negative numbers
    abs_data = np.sort(abs(data))
    abs_coord = np.absolute(coord)

    if len(coord) == 1: ##THIS ONE HAS TO BE TESTED
        start = max(np.where(abs_data<=abs_coord)[0])
        end = min(np.where(abs_data>=abs_coord)[0])

    else:
        min_val = min(abs_coord)
        max_val = max(abs_coord)

        start = max(np.where(abs_data<=min_val)[0])
        end = min(np.where(abs_data>=max_val)[0])

    return np.array((min(start, end), max(start, end)))


def get_depth_block(bbox, topo_file):
    '''
    Input:
        bbox (np.ndarray)       : 4x2 array with lat/lon float coordinates for the area of
                                  interest
                                  Coordinates should be top left, top right,
                                                        bottom right, bottom left
        topo_file (string)      : path to netcdf file containing the hydrodynamic data

    Returns:
        depth_area (np.ndarray) : (n,m) array of the depths within the bbox
        depth_func (func)       : interpolation function that outputs the depth within
                                  the array [+-width/2 (x), +-height/2 (y)]

    '''

    topo_depths = netcdf.NetCDFFile(topo_file)
    lat = topo_depths.variables['lat'][:].copy()
    lon = topo_depths.variables['lon'][:].copy()
    depths = topo_depths.variables['Band1'][:].copy()
    topo_depths.close()

    min_lat = min(bbox[:, 0])
    max_lat = max(bbox[:, 0])
    min_lon = min(bbox[:, 1])
    max_lon = max(bbox[:, 1])

    ##Finding the lon bounds is different cause the values are negative
    lon_idx = find_bound_idx(lon, [min_lon, max_lon])
    lon_idx = np.sort(lon.shape[0]-1-lon_idx)
    lat_idx = find_bound_idx(lat, [min_lat, max_lat])

    ##pull that data from the depth file:
    depths_area = depths[lat_idx[0]:lat_idx[-1]+1, lon_idx[0]:lon_idx[-1]+1]

    ##interpolation things:
    width = haversine(bbox[0,0], bbox[0,1], bbox[1,0], bbox[1,1])*1000 +100
    height = haversine(bbox[1,0], bbox[1,1], bbox[2,0], bbox[2,1])*1000 +100

    x = np.linspace(-width/2, width/2, depths_area.shape[0], endpoint = True)
    y = np.linspace(-height/2, height/2, depths_area.shape[1], endpoint = True)

    depths_func = RegularGridInterpolator((x, y), depths_area)

    return depths_area, depths_func


def get_current_block(bbox, current_file):
    '''
    Input:
        bbox (np.ndarray)     : 4x2 array with lon/lat float coordinates for the area of 
                                interest
                                Coordinates should be top left, top right,
                                                      bottom right, bottom left
                                Each bbox corner should be in [lon, lat] order
        current_file (string) : path to netcdf file containing the hydrodynamic data

    Returns:
        u_area (np.ndarray)   : (n,m) array of currents in the u direction within the bbox
        v_area (np.ndarray)   : (n,m) array of currents in the v direction within the bbox
        u_func (func)         : interpolation function that outputs the u value at 
                                the specified array[depth, x, y] point
        v_func (func)         : interpolation function that outputs the v value at 
                                the specified array[depth, x, y] point

    '''

    all_currents = netcdf.NetCDFFile(current_file)
    lon = all_currents.variables['lon'][:].copy()
    lon -= 360
    lat = all_currents.variables['lat'][:].copy()
    u = all_currents.variables['u'][:].copy()
    v = all_currents.variables['v'][:].copy()
    depths = all_currents.variables['depth'][:].copy()
    all_currents.close()

    ##find the bounds
    min_lon = min(bbox[:, 1])
    max_lon = max(bbox[:, 1])
    min_lat = min(bbox[:, 0])
    max_lat = max(bbox[:, 0])

    ##Finding the lon bounds is different cause the values are negative
    lon_idx = find_bound_idx(lon, [min_lon, max_lon])
    lon_idx = np.sort(lon.shape[0]-1-lon_idx)
    lat_idx = find_bound_idx(lat, [min_lat, max_lat])

    ##pull those coordinates from the current file
    u_area = (u[0, :, lat_idx[0]:lat_idx[-1]+1, lon_idx[0]:lon_idx[-1]+1])
    v_area = (v[0, :, lat_idx[0]:lat_idx[-1]+1, lon_idx[0]:lon_idx[-1]+1])

    ##Filter out currents that are greater than -9999:
    u_area[u_area<-800] = 0.0
    v_area[v_area<-800] = 0.0

    uv_area = np.hypot(u_area, v_area)

    ##This is all interpolation things
    ##allocate arrays for width, height
    width = haversine(bbox[0,0], bbox[0,1], bbox[1,0], bbox[1,1])*1000 +100
    height = haversine(bbox[1,0], bbox[1,1], bbox[2,0], bbox[2,1])*1000 +100
    
    ##create grid with lat-lon coordinates
    x = np.linspace(-width/2, width/2, u_area.shape[1], endpoint = True)
    y = np.linspace(-height/2, height/2, u_area.shape[2], endpoint = True)

    u_func = RegularGridInterpolator((depths, x, y), u_area)
    v_func = RegularGridInterpolator((depths, x, y), v_area)
    uv_func = RegularGridInterpolator((depths, x, y), uv_area)

    return u_area, v_area, uv_area, u_func, v_func, uv_func


def get_multi_current_block(bbox, current_file1, current_file2):
    '''
    Same as get_current_block but takes time into consideration

    Inputs:
        current_file1 (string) : path to netcdf file containing the hydrodynamic data
        current_file2 (string) : path to netcdf file containing the hydrodynamic data

    Returns:
        u_func (func)          : interpolation function that outputs the u value at the 
                                 specified time and location
                                 To get the u value call: ufunc([time in hrs, depth, x, y])
        v_func (func)          : interpolation function that outputs the v value at the 
                                 specified time and location
                                 To get the v value call: vfunc([time in hrs, depth, x, y])
        uv_func (func)         : interpolation function that outputs the hypotenuse value 
                                 of u and v at the specified time and location
                                 To get the uv value call: ufunc([time in hrs, depth, x, y])

    '''

    all_currents1 = netcdf.NetCDFFile(current_file1)
    lon = all_currents1.variables['lon'][:].copy()
    lon -= 360
    lat = all_currents1.variables['lat'][:].copy()
    u1 = all_currents1.variables['u'][:].copy()
    v1 = all_currents1.variables['v'][:].copy()
    depths = all_currents1.variables['depth'][:].copy()
    t1 = all_currents1.variables['time'][:][-1].copy()
    all_currents1.close()

    all_currents2 = netcdf.NetCDFFile(current_file2)
    u2 = all_currents2.variables['u'][:].copy()
    v2 = all_currents2.variables['v'][:].copy()
    t2 = all_currents2.variables['time'][:][-1].copy()
    all_currents2.close()

    ##find the bounds
    min_lon = min(bbox[:, 1])
    max_lon = max(bbox[:, 1])
    min_lat = min(bbox[:, 0])
    max_lat = max(bbox[:, 0])

    ##Finding the lon bounds is different cause the values are negative
    lon_idx = find_bound_idx(lon, [min_lon, max_lon])
    lon_idx = np.sort(lon.shape[0]-1-lon_idx)
    lat_idx = find_bound_idx(lat, [min_lat, max_lat])

    ##pull those coordinates from the current file
    u_area1 = (u1[:, :, lat_idx[0]:lat_idx[-1]+1, lon_idx[0]:lon_idx[-1]+1])
    v_area1 = (v1[:, :, lat_idx[0]:lat_idx[-1]+1, lon_idx[0]:lon_idx[-1]+1])

    u_area2 = (u2[:, :, lat_idx[0]:lat_idx[-1]+1, lon_idx[0]:lon_idx[-1]+1])
    v_area2 = (v2[:, :, lat_idx[0]:lat_idx[-1]+1, lon_idx[0]:lon_idx[-1]+1])

    u_area = np.vstack((u_area1, u_area2))
    v_area = np.vstack((v_area1, v_area2))

    ##Filter out currents that are greater than -9999:
    u_area[u_area<-800] = 0.0
    v_area[v_area<-800] = 0.0

    uv_area = np.hypot(u_area, v_area)

    ##This is all interpolation things
    ##allocate arrays for width, height
    width = haversine(bbox[0,0], bbox[0,1], bbox[1,0], bbox[1,1])*1000 +100
    height = haversine(bbox[1,0], bbox[1,1], bbox[2,0], bbox[2,1])*1000 +100
    
    ##create grid with lat-lon coordinates
    x = np.linspace(-width/2, width/2, u_area.shape[2], endpoint = True)
    y = np.linspace(-height/2, height/2, u_area.shape[3], endpoint = True)
    t = range(0, t1+t2)

    u_func = RegularGridInterpolator((t, depths, x, y), u_area)
    v_func = RegularGridInterpolator((t, depths, x, y), v_area)
    uv_func = RegularGridInterpolator((t, depths, x, y), uv_area)

    return u_func, v_func, uv_func


def grid_data(interp_f, x_bound, y_bound, spacing, z, time_hrs):
    '''
    Creates mesh grid based on dim size
    Used for visualizing data

    Input:
        interp_f (func)       : Scipy RegularGridInterpolator function that interpolates 
                                the data
                                Takes an array of coordinates as input
        x_bound (list)        : list of min-max values for the x values
        y_bound (list)        : list of min-max values for the y values
        spacing (int)         : how far apart the grid values should be
        z (np.ndarray)        : depth values for all of the input
        time_hrs (float)      : time in hours to take the currents

    Returns:
        interp_data (np.array): shape=(dim), interpolated data of the provided coordinates
    '''

    x = np.arange(x_bound[0], x_bound[1]+1, spacing)
    y = np.arange(y_bound[0], y_bound[1]+1, spacing)
    xv, yv = np.meshgrid(x,y)

    if len(z) == 0:
        grid = zip(xv.flatten(), yv.flatten())
    else:
        z = abs(z)
        grid = zip(z.flatten(), xv.flatten(), yv.flatten())

        if len(time_hrs) == 1:
            t = np.ones((len(grid), 1))*time_hrs
        else:
            t = time_hrs.flatten()
        grid = np.hstack((t, grid))

    ##feed it into the interpolation function to get output data of the same size
    interp_data = interp_f(grid)

    return interp_data.reshape(x.shape[0], y.shape[0])


def wpts_to_yaml(input_waypoints, save_path):
    '''
    TODO: Figure out how to format the points properly 

    Output a yaml file containing the inputted waypoints
    Used for the UUV Simulator

    '''
    all_wpt_dict = []
    for idx in range(input_waypoints.shape[0]):
        wpt_dict = {'point' : '{0}'.format(input_waypoints[idx].tolist()),
                    'max_forward_speed' : 0.4,
                    'heading' : 0,
                    'use_fixed_heading' : 'False'
                    }
        all_wpt_dict.append(wpt_dict)

    all_dict = dict(inertial_frame_id = 'world',
                    waypoints = all_wpt_dict)
    with open('test.yaml', 'w') as outfile:
        yaml.dump(all_dict, outfile, default_flow_style=False)


def load_currents():
    '''
    Initialize survey area bbox, u,v,depth functions

    Returns:
        place_bbox(np.ndarray) : dim=(4x2) lat/lon coordinates of specified map
        ufunc(function)        : time-dependent currents in u-direction. 
                                 Inputs are: (time in hrs, depth, x,y)
        vfunc(function)        : time-dependent currents in v-direction. 
                                 Inputs are (time in hrs, depth, x,y)
        dfunc(function)        : returns bottom depth at that coordinate. 
                                 Inputs are (x,y)

    '''

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

    all_locations = pickle.load( open(trash_finder_path + "/config/locations.p", "rb"))
    place_bbox = all_locations["mission_bay_flatter_bbox"]

    ufunc, vfunc, uvfunc = get_multi_current_block(place_bbox, 
                                                   currents_path1,
                                                   currents_path2)
    d_area, dfunc = get_depth_block(place_bbox, topo_path)

    return place_bbox, ufunc, vfunc, dfunc


def get_width(bbox):
    '''
    Returns width (float) in meters

    Inputs:
        bbox (np.ndarray) : lat/lon coordinates
    Returns:
        width (float)     : in meters

    '''
    return haversine(bbox[0,0], bbox[0,1], bbox[1,0], bbox[1,1]) * 1000

def get_height(bbox):
    '''
    Returns height (float) in meters

    Inputs:
        bbox (np.ndarray) : lat/lon coordinates
    Returns:
        height (float)    : in meters

    '''
    return haversine(bbox[1,0], bbox[1,1], bbox[2,0], bbox[2,1]) * 1000


def filter_edges(waypoints, env_edge, col_idx):
    '''
    Used to limit the waypoints to the edges of the map
    This specifically limits only one column at a time

    waypoints (np.ndarray) : dim=nx3 array of points
    env_edge (float)       : Either env.height/2, env.width/2, or env.max_depth
    col_idx (int)          : Used to indicate which column to filter (ex = x,y,z position)

    Returns
        waypoints (np.ndarray) : array of points with single column within limits of +/-env_edge

    '''
    found_idx = abs(waypoints[:, col_idx]) > env_edge
    waypoints[found_idx, col_idx] = env_edge*np.sign(waypoints[found_idx, col_idx])
    return waypoints    


def fix_waypoint_edges(waypoints, env):
    '''
    Used to limit the waypoints to the edges of the map

    waypoints (np.ndarray) : dim=nx3 array of points
    env (object)           : trash_utils/Env.py object to access
                             (width, height, max_depth)

    Returns:
        waypoints(np.ndarray) : array of points within map limits

    '''
    waypoints = filter_edges(waypoints, env.width/2, 0)
    waypoints = filter_edges(waypoints, env.height/2, 1)
    waypoints = filter_edges(waypoints, env.max_depth, 2)

    return waypoints



def main():
    rospack = rospkg.RosPack()
    trash_finder_path = rospack.get_path("trash_finder")
    data_path = rospack.get_path("data_files")
    config = pickle.load(open(trash_finder_path + "/config/demo_configs.p", "rb"))

    currents = config['current_file']
    currents_path = data_path + '/' + currents
    currents1 = config['current_file1']
    currents_path1 = data_path + '/' + currents1
    currents2 = config['current_file2']
    currents_path2 = data_path + '/' + currents2
    topo = config['topo_file']
    topo_path = data_path + '/' + topo

    places = np.array(config['coords_bbox'])
    print ("PLACES: ", places)

    # get_current_block(places, currents_path)
    # get_depth_block(places, topo_path)
    ufunc, vfunc, uvfunc = get_multi_current_block(places, currents_path1, currents_path2)