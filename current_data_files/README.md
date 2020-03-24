# Topological and Current data

Contains the topological and hydrodynamic data used for the experiment and files to parse/visualize the data

## /Data/:

Topological data files:
	southern_calif_crm_v1.nc: Data of Southern California. Taken from: the [NOAA Bathymetry maps](https://maps.ngdc.noaa.gov/viewers/bathymetry/?layers=dem)

Hydrodynamic data files:
	ca_subCA_das_2020010615.nc: Data of all of California. Taken from: the [Joint Institution for Regional Earth System Science and Engineering](https://www.sccoos.org/data/roms-3km/)

## Files:
 - **generate_stl_file.py** - Generates a topological map (STL file) that can be used for the bottom of a Gazebo environment
 - **haversine_dist.py** - Helper file for calculating km distances between two lat/lon coords
 - **topological_and_hydro_visualization** Jupyter notebook for visualizing topological and hydrodynamic data