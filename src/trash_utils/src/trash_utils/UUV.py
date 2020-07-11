from numpy import array

class UUV():

	def __init__(self, init_pos = array([0.0, 0.0, 0.0]), init_heading_rad = 0):
		self.pos = init_pos
		self.heading_rad = init_heading_rad

		##TODO: Figure out what to do about this section
		self.max_search_thrust = 1.0	 		##max 2knots?
		self.max_thrust = 7.0   				##m/sec 

		self.trash_detection_dist = 5
		self.found_trash = []	##Feel free to change this as needed

		self.E_appr = 75 ##Approximate, minimum energy per sampling time consumed by the vehicle at each time step