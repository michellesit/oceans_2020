import sys
import pickle

import rospkg
import numpy as np

from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter
from scipy.io import netcdf

import matplotlib.pyplot as plt
from matplotlib import cm
from stl import mesh, Mode
import matplotlib.tri as mtri

from trash_utils.haversine_dist import haversine

'''
Generates the real-world topological map for a Gazebo environment
Takes a topological file (NetCDF) and creates a STL file which can be used
	for the bottom of a Gazebo environment

To see the generated 3D map, run this file using python3

'''

def get_total_area(bbox):
    '''
	Calculates the width, height, total search area, and approximate time to search
		entire area at 5 knots (9.26 km/hr)

    Arguments:
		bbox (np.ndarray) = 4x2 np array filled with float coordinate values

    Returns:
		width (float) = in meters
		height (float) = in meters
    '''
    
    [lat1, lon1] = bbox[0, :]
    [lat2, lon2] = bbox[1, :]
    [lat3, lon3] = bbox[2, :]
    [lat4, lon4] = bbox[3, :]

    ##Calculate the haversine dist between all the points:
    ##loc1 to loc2 (width)
    width = haversine(lat1, lon1, lat2, lon2)
    ##loc2 to loc 3 (height)
    height = haversine(lat2, lon2, lat3, lon3)
    
    print ("Width (km): ", width)
    print ("Height (km) ", height)
    
    total_area = height*width
    print ("Total area in km: ", total_area)
    
    ##5 knots = 9.26 km/hr
    
    total_search_days = total_area/(9.26*24)
    total_search_hours = total_area/9.26
    
    print ("Total search time (days): ", total_search_days)
    print ("Total search time (hours): ", total_search_hours)
    
    return width*1000, height*1000


def parse_data(topological_path, bbox_coords):
	'''
	Grabs depth data from the topological file using the bbox coords

	Arguments:
		topological_path (String): path to the NetCDFFile containing the topological data
		bbox_coords (np.ndarray): 4x2 numpy array with float lat/lon coordinates for the
								  search area

	Return:
		x (np.ndarray): n x m array x coord in lon zeroed to origin
		y (np.ndarray): n x m array y coord in lat zeroed to origin
		z (np.ndarray): (n*m) x 1 array depth value (m) for each x,y coordinate

	'''

	socal = netcdf.NetCDFFile(topological_path)
	xog = np.copy(socal.variables['lon'][:])
	yog = np.copy(socal.variables['lat'][:])
	zog = np.copy(socal.variables['Band1'][:])
	socal.close()

	width, height = get_total_area(bbox_coords)

	min_lon = min(bbox_coords[:,1])
	max_lon = max(bbox_coords[:,1])
	xfilter = np.argwhere((xog <= max_lon) & (xog >= min_lon))
	xclip = xog[xfilter].flatten()
	xclip -= min(xclip)
	xclip /= np.max(np.abs(xclip), axis=0)
	xclip *= width
	xclip -= np.max(xclip)/2

	min_lat = min(bbox_coords[:,0])
	max_lat = max(bbox_coords[:,0])
	yfilter = np.argwhere((yog >= min_lat) & (yog<= max_lat))
	yclip = yog[yfilter].flatten()
	yclip -= min(yclip)
	yclip /= np.max(np.abs(yclip), axis=0)
	yclip *= height
	yclip -= np.max(yclip)/2

	x, y = np.meshgrid(xclip, yclip)
	z = zog[yfilter[0][0]:yfilter[-1][0]+1, xfilter[0][0]:xfilter[-1][0]+1].flatten()

	return x,y,z


def generate_stl(x, y, z, savefile_name, savefig=False):
	'''
	Generates the meshes needed for the stl file

	NOTE: Gazebo will import your mesh in meters

	Arguments:
		x (np.ndarray): n x m array x coord in lon zeroed to origin
		y (np.ndarray): n x m array y coord in lat zeroed to origin
		z (np.ndarray): (n*m) x 1 array depth value (m) for each x,y coordinate
		savefile_name (String): path and name to save file
		savefig (bool): saves the 3d mesh or not

	'''

	# Scale the surface for this example
	# z *= 0.05
	# Remember that Gazebo uses ENU (east-north-up) convention, so underwater
	# the Z coordinate will be negative
	# z -= 3

	# Point clouds usually don't come in nice grids, so let's make it a (N, 3)
	# matrix just to show how it can be done. If you have outliers or noise, you should
	# treat those values now.
	# xyz = np.zeros(shape=(x.size, 3))
	xyz = np.zeros(shape=(z.shape[0], 3))
	xyz[:, 0] = x.flatten()
	xyz[:, 1] = y.flatten()
	xyz[:, 2] = z.flatten()

	# Generate a grid for the X and Y coordinates, change the number of points
	# to your needs. Large grids can generate files that are too big for Gazebo, so
	# be careful when choosing the resolution of your grid.
	x_grid, y_grid = np.meshgrid(np.linspace(xyz[:, 0].min(), xyz[:, 0].max(), 500),
	                             np.linspace(xyz[:, 1].min(), xyz[:, 1].max(), 300))

	# Interpolate over the point cloud for our grid
	z_grid = np.flipud(griddata(xyz[:, 0:2], xyz[:, 2], (x_grid, y_grid),
	                  method='linear'))

	# Option to treat noise
	#z_grid = gaussian_filter(z_grid, sigma=1)

	# Show the resulting heightmap as an image
	fig = plt.figure(figsize=(8, 6))
	ax = fig.add_subplot(111)
	im = plt.imshow(z_grid)
	fig.colorbar(im)
	plt.show(block=True)
	plt.close()

	# Flatten our interpolated data for triangulation
	output = np.zeros(shape=(x_grid.size, 3))
	output[:, 0] = x_grid.flatten()
	output[:, 1] = y_grid.flatten()
	output[:, 2] = z_grid.flatten()

	# Triangulation of the interpolated data
	tri = mtri.Triangulation(output[:, 0], output[:, 1])

	# Show the resulting surface
	# NOTE: This graph will only show properly with python3
	#       Otherwise this graph will be blank/throws an error
	if sys.version_info[0] >= 3:
		print ("PYTHON3: Showing 3D mesh")
		fig2 = plt.figure(figsize=(8, 6))
		ax2 = fig2.add_subplot(111, projection='3d')
		ax2.plot_trisurf(tri, output[:, 2], cmap=plt.cm.CMRmap, shade=True, linewidth=0.1)
		# ax2.axis('equal')

		plt.show(block=True)
		plt.close()

	else:
		print("PYTHON2 VERSION. Cannot show the 3D mesh. Run this file using Python3 to see.")

	# Create the mesh object
	seabed_mesh = mesh.Mesh(np.zeros(tri.triangles.shape[0], dtype=mesh.Mesh.dtype))

	# Set the vectors
	for i, f in enumerate(tri.triangles):
	    for j in range(3):
	        seabed_mesh.vectors[i][j] = output[f[j]]

	# Store the seabed as a STL file
	if savefig:
		seabed_mesh.save(savefile_name)


def main():
	'''
	loc1-4 should be the bbox of interest. Should go clockwise starting from top left
	loc1 List[float, float] = top left
	loc2 List[float, float] = top right
	loc3 List[float, float] = bottom right
	loc4 List[float, float] = bottom left
	'''

	rospack = rospkg.RosPack()
	trash_finder_path = rospack.get_path('trash_finder')
	data_path = rospack.get_path('data_files')

	# topological_path = data_path+'/southern_calif_crm_v1.nc'
	topological_path = data_path+'/crm_socal_1as_vers2.nc'
	# topological_path = data_path+'/san_francisco_13_navd88_2010.nc'

	all_locations = pickle.load(open(trash_finder_path+"/config/locations.p", "rb"))
	bbox_coords = all_locations["mission_bay_flatter_bbox"]

	savefile_name = 'nosave.stl'
	savefig = False

	x, y, z = parse_data(topological_path, bbox_coords)
	generate_stl(x, y, z, savefile_name, savefig)

main()
