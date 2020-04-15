from scipy.io import netcdf
import numpy as np
import matplotlib.pyplot as plt

import rospkg
from trash_utils.finder_utils import get_current_block

import pdb

##Visualize the current maps
##get bbox coordinates from the netcdf file at all depths
##Filter out the currents that are greater than 9999999
##Visualize the currents at each depth
def visualize_currents(bbox, current_file, visualize=True):
    '''
    

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
    if visualize == True:
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


def visualize_depths(bbox, topo_file, visualize=True):
    min_lat = min(bbox[:, 0])
    max_lat = max(bbox[:, 0])
    min_lon = min(bbox[:, 1])
    max_lon = max(bbox[:, 1]) 


##hydrodynamic current
rospack = rospkg.RosPack()
data_path = rospack.get_path("data_files")
print ('data_path: ', data_path)

current_path = data_path+'/ca_subCA_das_2020010615.nc'
current_data = netcdf.NetCDFFile(current_path)
depth = current_data.variables['depth'][:].copy()
current_data.close()



## coordinates
##MBay flatter
## 'Width (km): ', 2.150556797649856)
## 'Height (km) ', 2.1127036062467672)
loc1 = [32.766, -117.279]
loc2 = [32.766, -117.256]
loc3 = [32.747, -117.256]
loc4 = [32.747, -117.279]

place_bbox = np.array((loc1, loc2, loc3, loc4))
visualize_currents(place_bbox, current_path)