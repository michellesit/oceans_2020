import os
import pickle
import rospkg

'''
Creates the demo config paramters in a pickle file located at /trash_finder/config/
Change the parameters as needed below
'''


def pickling_info(world_env, current_file, topo_file,
					num_soda_cans, distribution_type,
					x_bounds, y_bounds, z_bounds, z_dist,
					gaussian_mu, gaussian_sigma,
					sc_coord_file):
	'''
	Inputs:
		world_env (str) = "mission_bay_flatter", "lajolla_cove_south, "san_fran_shoal" 

		current_file (str) = "ca_subCA_das_2020010615.nc"
			Netcdf file containing the hydrodynamic data.
			Needs to exist in the /src/data_files folder

		topo_file (str) = "crm_socal_1as_vers2.nc", "san_francisco_13_navd88_2010.nc"
			Netcdf file containing the bathymetry data. 
			Needs to exist in the /src/data_files folder

		num_soda_cans (int) = number of soda cans to generate
			This number gets squared if distribution_type is "file"
		distribution_type (str) = "random", "gaussian", "file"
			Fill out the corresponding section depending on your choice
			Random distribution: soda cans randomly distributed throughout map
			Gaussian distribution: soda cans distributed from a single point
			File distribution: soda can locations manually provided via
				the man_create_soda_pos.py file

		x_bounds (array of int) = x coordinate range for distribution
		y_bounds (array of int) = y coordinate range for distribution
		z_bounds (array of int) = (optional) z coordinate range for distribution
		z_dist (str) = "depth_file", "grid", "hand"
			depth_file : depth is the bottom of the seafloor according to the topo_file
			grid : depth is generated randomly using z_bounds values
			hand: values written in by the user in man_create_soda_pos.py

		sc_coord_file = TODO: what is this?

	'''
	rospack = rospkg.RosPack()
	trash_finder_path = rospack.get_path('trash_finder')
	data_path = rospack.get_path('data_files')


	##check world_env exists
	if world_env not in ['mission_bay_flatter', 'lajolla_cove_south', 'san_fran_shoal']:
		raise Exception ("WORLD_ENV IS NOT VALID")
	
	all_locations = pickle.load(open(trash_finder_path+"/config/locations.p", "rb"))
	coords_bbox = all_locations[world_env+"_bbox"]

	##check current/topo files exist
	if not os.path.isfile(data_path + "/" + current_file):
		raise Exception ("CURRENT FILE DOES NOT EXIST IN DATA_FILES FOLDER")
	if not os.path.isfile(data_path + "/" + topo_file):
		raise Exception ("BATHYMETRY FILE DOES NOT EXIST IN DATA_FILES FOLDER")

	##check num_soda_cans is reasonable
	if not (num_soda_cans >= 0):
		raise Exception ("CANNOT PROVIDE A NONNEGATIVE VALUE FOR NUMBER OF SODA CANS")

	##check distribution_type is valid
	if distribution_type not in ['random', 'gaussian', 'file']:
		raise Exception ("DISTRIBUTION TYPE IS NOT VALID")

	##TODO: check x_bounds, y_bounds reasonable
	map_width = all_locations[world_env+'_width'] * 1000
	map_height = all_locations[world_env+'_height'] * 1000
	

	##TODO: check z_dist. check z_bounds reasonable
	if z_dist not in ['depth_file', 'grid', 'hand']:
		raise Exception ("Z_DIST IS NOT VALID")


	##TODO: check gaussian_mu reasonable

	##TODO: check gaussian_sigma reasonable


	##TODO: check sc_file exists
	# if not os.path.isfile(data_path + "/" + current_file):
	# 	raise Exception ("CURRENT FILE DOES NOT EXIST IN DATA_FILES FOLDER")


	demo_configs = {
		##Demo
		"world_env" : world_env,
		"coords_bbox" : coords_bbox,

		##Currents
		"current_file" : current_file,
		"current_file1" : current_file1,
		"current_file2" : current_file2,

		##Topo
		"topo_file" : topo_file,

		##Soda_launch
		"num_soda_cans" : num_soda_cans,
		"distribution_type" : distribution_type,

		##Soda can distribution params
		##All/Rand
		"x_bounds" : x_bounds,
		"y_bounds" : y_bounds,
		"z_bounds" : z_bounds,

		##File
		"z_dist" : z_dist,

		##Gauss
		"gaussian_mu" : gaussian_mu,
		"gaussian_sigma" : gaussian_sigma,

		##SC_file
		"sc_coord_file" : sc_coord_file
	}

	pickle.dump(demo_configs, open(trash_finder_path+'/config/demo_configs.p', "wb"))


if __name__ == '__main__':
	##Specify all these values:

	world_env = "mission_bay_flatter"
	current_file = "ca_subCA_das_2020010615.nc"
	current_file1 = "ca_subCA_fcst_2020051703.nc"
	current_file2 = "ca_subCA_fcst_2020051803.nc"
	topo_file = "crm_socal_1as_vers2.nc"

	num_soda_cans = 3
	distribution_type = "file"

	x_bounds = [0, 100]
	y_bounds = [0, 100]
	z_bounds = [0, 40]
	z_dist = "depth_file"

	gaussian_mu = 0.0
	gaussian_sigma = 0.0

	sc_coord_file = "test_sc_pts.yaml"

	pickling_info(world_env, current_file, topo_file,
					num_soda_cans, distribution_type,
					x_bounds, y_bounds, z_bounds, z_dist,
					gaussian_mu, gaussian_sigma,
					sc_coord_file)