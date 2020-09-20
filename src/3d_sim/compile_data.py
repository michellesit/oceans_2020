import glob
import csv
import pdb

import numpy as np

##Grab energy files
energy_files = []
all_files = glob.glob('./santa_monica_costs/*.csv')

for f in all_files:
	full_file_name = f.split("/")
	name = full_file_name[-1].split("_")

	if name[1] == "energy":
		energy_files.append(f)


##take data from csv file and compile into matrix
##Energy matrix should be (hspt1, hspt2, time)
num_hspts = len(energy_files)
energy_files = sorted(energy_files)

for f_idx in range(len(energy_files)):
	print ("FOR F_IDX: ", f_idx)
	with open(energy_files[f_idx]) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=",")
		row_idx = 0

		for row in csv_reader:
			##set the z dimension of all_energy_costs
			if f_idx == 0 and row_idx == 0:
				print ("only doing htis once")
				z_dim = len(row[1:])
				all_energy_costs = np.ones((num_hspts, num_hspts, z_dim)) * 99999999

			if f_idx == row_idx:
				row_idx += 1
				print ("skipping this col real quick. row_idx now: ", row_idx)

			if row[0] == 'Energy_cost':
				print ("ROW_IDX: ", row_idx)
				costs = np.array(row[1:]).astype("float")
				all_energy_costs[f_idx, row_idx, :] = costs

				row_idx += 1

print (all_energy_costs)
with open('santa_monica_energy_costs.npy', 'wb') as f:
	np.save(f, all_energy_costs)