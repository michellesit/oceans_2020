class UUV():

	def __init__(self, init_pos = [0,0,0], init_heading_rad = 0):
		self.pos = init_pos
		self.heading_rad = init_heading_rad

		##TODO: Figure out what to do about this section
		self.max_search_thrust = 1.0	 		##max 2knots?
		self.max_thrust = 7.0   				##m/sec 

		self.found_trash = []	##Feel free to change this as needed