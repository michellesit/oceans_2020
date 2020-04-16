#!/usr/bin/env python
import pickle

import rospkg
from lxml import etree
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

	##Edit the launch file to create some number of soda cans "robots"
	parser = etree.XMLParser(remove_blank_text=True)
	tree = etree.parse(trash_worlds_path+"/launch/basic_demo.launch", parser)
	launch = tree.getroot()

	##Edit the launch file with the right world
	edit_launch = launch.getchildren()[8]
	edit_include = edit_launch.attrib
	launch_file = edit_include['file']
	split = launch_file.split('/')
	new_launch = split[0] + "/" + split[1] + "/" + config["world_env"] + ".launch"
	edit_include['file'] = new_launch

	##Add in all the unique soda can hydrodynamic topics
	for i in range (num_soda_cans):
		include_file = etree.SubElement(launch, "include")
		include_file.set("file", "$(find soda_can_description)/launch/single_soda.launch")

		sc_x = etree.SubElement(include_file, "arg")
		sc_x.set("name", "x")
		sc_x.set("default", str(sc_pos[0, i]))

		sc_y = etree.SubElement(include_file, "arg")
		sc_y.set("name", "y")
		sc_y.set("default", str(sc_pos[1, i]))

		sc_z = etree.SubElement(include_file, "arg")
		sc_z.set("name", "z")
		sc_z.set("default", str(sc_pos[2, i]))

		sc_yaw = etree.SubElement(include_file, "arg")
		sc_yaw.set("name", "yaw")
		sc_yaw.set("default", "0")

		sc_ns = etree.SubElement(include_file, "arg")
		sc_ns.set("name", "namespace")
		sc_ns.set("value", "soda_can{0}".format(i))

	##write the xml file
	##begin generating the text to write into the file
	tree.write(trash_worlds_path+"/launch/demo_with_cans.launch", pretty_print=True)


	##Also update the lj.world file so the hydrodynamics forecasts match the number of soda cans created here
	world_tree = etree.parse(trash_worlds_path+"/worlds/{0}.world".format(config["world_env"]), parser)
	sdf = world_tree.getroot()
	world = sdf.getchildren()[0]

	for i in range (num_soda_cans):
		plugin_file = etree.SubElement(world, "plugin")
		plugin_file.set("name", "underwater_current_plugin")
		plugin_file.set("filename", "libuuv_underwater_current_ros_plugin.so")

		ns = etree.SubElement(plugin_file, "namespace")
		ns.text = "soda_can{0}".format(i)

		cc = etree.SubElement(plugin_file, "constant_current")

		topic = etree.SubElement(cc, "topic")
		topic.text = "current_velocity"

	world_tree.write(trash_worlds_path+"/worlds/demo_with_cans.world", pretty_print=True)


if __name__ == '__main__':
	create_soda_launch_files()
