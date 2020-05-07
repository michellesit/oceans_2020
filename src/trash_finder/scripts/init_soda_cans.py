#!/usr/bin/env python
import pickle

import rospkg
from lxml.etree import XMLParser, parse, SubElement
import numpy as np
from numpy.random import randint, normal

from man_create_soda_pos import man_create_soda_pos
'''
Each soda can in the simulation needs to subscribe to its own hydrodynamic topic.
Since each can needs a unique name and topic name in the launch files, this file 
	uses /trash_worlds/launch/basic_demo.launch and /trash_worlds/worlds/ as templates.
	Appends the soda can objects and topic names to the end of those files.
The resulting files are named demo_with_cans.launch and demo_with_cans.world in
	their respective folders

Types of distribution:
	random - soda cans are randomly distributed
	file - soda cans are manually created via man_create_soda_pos
	gaussian - gaussian distribution of soda cans around a single point

To run:
python init_soda_cans.py

'''

def create_soda_launch_files():
	rospack = rospkg.RosPack()
	trash_worlds_path = rospack.get_path("trash_worlds")
	trash_finder_path = rospack.get_path("trash_finder")
	data_path = rospack.get_path("data_files")

	config_path = trash_finder_path + "/config/"
	config = pickle.load(open(config_path + 'demo_configs.p', "rb"))

	##Get info about number of soda cans and their distribution type
	num_soda_cans = config['num_soda_cans']
	dist_type = config['distribution_type']
	if dist_type == 'gaussian':
		##Gaussian distribution/Centered around a point
		mu = config['gaussian_mu']
		sigma = config['gaussian_sigma']

		xy_pos = normal(mu, sigma, size=(2, num_soda_cans))
		z_pos = -randint(z_range[0], z_range[1], size=(num_soda_cans))
		sc_pos = np.vstack((xy_pos, z_pos))

	elif dist_type == 'file':
		##Manually specified trash distribution
		topo_path = data_path + '/' + config['topo_file']
		bbox = np.array(config['coords_bbox'])

		x_range = config['x_bounds']
		y_range = config['y_bounds']
		z_range = config['z_bounds']
		z_dist = config['z_dist']

		sc_pos = man_create_soda_pos(num_soda_cans, bbox, topo_path,
									x_range, y_range, z_range, z_dist)
		if z_dist == "depth_file":
			num_soda_cans = num_soda_cans*num_soda_cans

	else:
		##Randomly generated trash within a specified bound
		x_range = config['x_bounds']
		y_range = config['y_bounds']
		z_range = config['z_bounds']

		sc_pos_x = randint(x_range[0], x_range[1], size=(num_soda_cans))
		sc_pos_y = randint(y_range[0], y_range[1], size=(num_soda_cans))
		sc_pos_z = -randint(z_range[0], z_range[1], size=(num_soda_cans))
		sc_pos = np.vstack((sc_pos_x, sc_pos_y, sc_pos_z))

	##Edit the world file to create some number of soda cans "robots"
	parser = XMLParser(remove_blank_text=True)
	world_tree = parse(trash_worlds_path+"/worlds/{0}.world".format(config["world_env"]), parser)
	sdf = world_tree.getroot()
	world = sdf.getchildren()[0]

	for i in range (num_soda_cans):
		population = SubElement(world, "population")
		population.set("name", "can_population_{0}".format(i))

		model = SubElement(population, "model")
		model.set("name", "can{0}".format(i))

		include_param = SubElement(model, "include")
		static_bool = SubElement(include_param, "static")
		static_bool.text = "false"
		uri = SubElement(include_param, "uri")
		uri.text = "model://coke_can"

		pose = SubElement(population, "pose")
		pose.text = "{0} {1} {2} 0 0 0".format(int(sc_pos[0,i]), int(sc_pos[1,i]), int(sc_pos[2,i]))

		box = SubElement(population, "box")
		size = SubElement(box, "size")
		size.text = "5 5 3"

		model_count = SubElement(population, "model_count")
		model_count.text = "1"

		distribution_param = SubElement(population, "distribution")
		type_param = SubElement(distribution_param, "type")
		type_param.text = "random"

	world_tree.write(trash_worlds_path+"/worlds/demo_with_cans.world", xml_declaration=True, encoding='UTF-8', pretty_print=True)


if __name__ == '__main__':
	create_soda_launch_files()
