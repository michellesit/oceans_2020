import rospy
import xml.etree.ElementTree as e

from lxml import etree

# rospy.wait_for_service(f'/{self.uuv_name}/set_current_velocity/')

##lxml attempt
tree = e.parse("./../../trash_worlds/launch/lajolla_demo.launch")
launch = tree.getroot()

sc_pos = [10, 40, 10, 0.5, 1]

include_file = e.SubElement(launch, "include")
include_file.set("file", "$(find trash_worlds)/launch/single_soda.launch")

sc_x = e.SubElement(include_file, "arg")
sc_x.set("default", str(sc_pos[0]))
sc_x.set("name", "x")

# sc_y = e.SubElement(include_file, "arg")
# sc_y.set("name", "y", "default", sc_pos[1])

# sc_z = e.SubElement(include_file, "arg")
# sc_z.set("name", "z", "default", sc_pos[2])

# sc_y = e.SubElement(include_file, "arg")
# sc_y.set("name", "yaw", "default", sc_pos[3])

# sc_y = e.SubElement(include_file, "arg")
# sc_y.set("name", "namespace", "default", sc_pos[4])


##write the xml file
##begin generating the text to write into the file
tree.write("test.launch")

##Instead of rewriting the whole file, just add to the section at the end


# num_soda_cans = 20

# def generate_data_points(self):
# 	for x in range(num_soda_cans):
