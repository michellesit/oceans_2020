# Trash Finder

## To run the simulation:
 1. Edit pickle_config.py with the appropriate location name, number of soda cans, soda can distribution type, and parameters
 2. Create the pickle config file for the demo in /trash_finder/configs/:

 ```
 python pickle_config.py
 ```

 3. Generate the appropriate launch files for the demo with the soda can info:
 ```
 python init_soda_cans.py 
 ```

 4. To launch the world with the uuv and soda cans:
 ```
 roslaunch trash_worlds demo.launch
 ```
 

## Scripts

 - **pickle_configs.py** - Creates a pickle file from the demo configs dictionary
 - **init_soda_cans.py** - Uses /trash_worlds/launch/basic_demo.launch as a template file to create the demo launch file containing all the unique soda can hydrodynamic topics and their x,y,z positions, as well as the uuv.
 - **search.py** - Runs a 2D simulation of the environment and calculates the lowest cost path to travel it. Saves the optimal waypoints as a YAML file in /trash_finder/config/
 - **trash_sim.py** - Visualizes an approximation of the trash flow based on currents
 - **update_trash_states.py** - Uses Gazebo model states to apply a twist msg to the trash and UUV based on their position and local currents. Used for the ROS demo.

 - **uuv_local_current.py** - IN PROGRESS | TODO: FIX UPDATE MESSAGE
 

Helper files:
 - **man_create_soda_pos.py** - Generates soda can positions manually

 - **Topological_Map.py** - IN PROGRESS


## Launch
 - **oceans2020_demo.launch** - Modified default file from uuv_simulator. Tests basic waypoint following for a single uuv. (TODO: Change this name | UPDATE TO NEW FILE)
 - **send_waypoints_file.launch** - Default file from uuv_simulator. Sends the waypoints to the uuv to execute. Currently sends mowing_lawn_waypoints.yaml file.

 - **produce_hydrodynamic_forces.launch** - Helper file to subscribe a robot/soda can to its individual hydrodynamic forecast based on its local position
 - **update_trash_states.launch** - Helper file to subscribe the UUV/soda can to its individual hydrodynamic forecast based on its local position.

 - **check_pos.launch** - TODO: NOT SURE IF RELEVANT


## Config
Contains waypoint yaml files for the demos and config parameters in pickle format.

 - **demo_configs.p** - configs for the demo. Generated from /trash_finder/scripts/pickle_configs.py
 - **mowing_lawn_waypoints.yaml** - simple mowing the lawn pattern for testing the environment
 - **locations.p** - picke config file containing the search location bounding box. Generated from /oceans_2020/current_data_files/pickle_locations.py

 - **oceans2020_demo.yaml** - IN PROGRESS. NOT SURE IF RELEVANT