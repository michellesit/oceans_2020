from lxml import etree
from numpy.random import randint, normal

from sys import argv
##Instead of rewriting the whole file, just add to the section at the end

num_soda_cans = int(argv[1])

##randomly generate positions
sc_pos = randint(300, size=(num_soda_cans, 3))

##Gaussian distribution/Centered around a point
# mu = sys.argv[2]
# sigma = sys.argv[3]
# sc_pos = normal(mu, sigma, size=(num_soda_cans, 3))

sc_pos[:, 2] *= -1  ##Make depth negative

##Edit the launch file to create some number of soda cans "robots"
parser = etree.XMLParser(remove_blank_text=True)
tree = etree.parse("./../../trash_worlds/launch/lajolla_demo.launch", parser)
launch = tree.getroot()

for i in range (num_soda_cans):
	include_file = etree.SubElement(launch, "include")
	include_file.set("file", "$(find trash_worlds)/launch/single_soda.launch")

	sc_x = etree.SubElement(include_file, "arg")
	sc_x.set("name", "x")
	sc_x.set("default", str(sc_pos[i, 0]))

	sc_y = etree.SubElement(include_file, "arg")
	sc_y.set("name", "y")
	sc_y.set("default", str(sc_pos[i, 1]))

	sc_z = etree.SubElement(include_file, "arg")
	sc_z.set("name", "z")
	sc_z.set("default", str(sc_pos[i, 2]))

	sc_yaw = etree.SubElement(include_file, "arg")
	sc_yaw.set("name", "yaw")
	sc_yaw.set("default", "0")

	sc_ns = etree.SubElement(include_file, "arg")
	sc_ns.set("name", "namespace")
	sc_ns.set("default", "soda_can{0}".format(i))

##write the xml file
##begin generating the text to write into the file
tree.write("./../../trash_worlds/launch/lajolla_demo_with_sc.launch", pretty_print=True)


##Also update the lj.world file so the hydrodynamics forecasts match the number of soda cans created here
world_tree = etree.parse("./../../lajolla_world/worlds/lj.world", parser)
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

world_tree.write("./../../lajolla_world/worlds/lj_demo.world", pretty_print=True)