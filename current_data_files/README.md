# Analyzes the topological and current data

Analyzes the topological and hydrodynamic data for the demo. Mostly used for exploring through the data and creating the stl files needed for the gazebo environment. Start here to create the stl topological file for an area off the coast of California.

The raw data files can be found in ../data_files/

## Files:
 - **generate_stl_file.py** - Generates a topological map (STL file) that can be used for the bottom of a Gazebo environment
 - **visualize.py** - Visualization methods for currents and bathymetry maps
 - **pickle_locations.py** - Pickles the dictionary containing the survey location coordinates, widths (km), and heights (km)
 - **locations.p** - Pickle file containing survey location coordinates, widths (km), and heights (km)

## Create an environment
Edit generate_stl_file.py with the lat/lon decimal coordinates of your bbox area, and the save file name. Then run:
```
python generate_stl_file.py
```
Run it with python3 to see the 3D map. Otherwise, it'll just show you the 2D map.
