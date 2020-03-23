# oceans_2020 abstract demo

### Installation/Setting up Environment
Shengye's wifi environment requries Kinetic and Gazebo 9. I have setup a dockerfile to install the environment and necessary packages. It uses Shengye's ros-kinetic-gazebo9 image as its base. The image is built assuming you will mount the files from the home desktop to the container.

```
cd /oceans_2020/oceans_docker/
sudo docker build -t oceans_demo:mar23 .
```

Please shoot me an email if you run into any issues!


#### Building from script
I have included a script (./oceans_docker/install_uuv_pkgs.sh) to install the environment.
```
git clone https://github.com/michellesit/oceans_2020.git
cd /oceans_docker/
chmod +x install_uuv_pkgs.sh
./install_uuv_pkgs.sh

```

Otherwise you can go through the individual steps below if you run into any trouble.

#### Building from source

Info on how to install to set-up the lake environment: https://uuvsimulator.github.io/installation/#installation
```
sudo apt install ros-kinetic-uuv-simulator
```

Make sure you install catkin_tools:
```
sudo apt-get update
sudo apt-get install python-catkin-tools
```

Then download the rexrov2 simulator into `~/catkin_ws/src`: https://github.com/uuvsimulator/rexrov2

Download the uuv-simulator into `~/catkin_ws/src`: https://github.com/uuvsimulator/uuv_simulation_evaluation

(OPTIONAL) From https://github.com/CogRob/cogrob_ros, clone the whole repo and link the folders to src:
```
(after you clone CogRob outside of Oceans_2020)
cd /oceans_2020/src
ln -s ~/cogrob_ros/fetch_gazebo_wifi/ .
ln -s ~/cogrob_ros/gazebo_wifi_plugin/ .
```

You should have the following structure:
```
catkin_ws/
    current_data_files/
    oceans_docker/
    src/
        fetch_gazebo_wifi/
        gazebo_wifi_plugin/
        lajolla_world/
        rexrov2/
        soda_can_description/
        trash_finder/
        trash_worlds/
        uuv_simulation_evaluation/
        uuv_simulator/
```

Then in your ~/catkin_ws do:
```
catkin build
```

Test that your environment works by following the 'Follow Waypoints Simulation' section below.


### (OPTIONAL) Adaptive Information Sampling

To run the heatmap:
1. Open a terminal and run the launch file:
```
cd /catkin_ws/src/
roslaunch fetch_gazebo_wifi simulation.launch
```

2. Open another terminal and run rviz:
```
rviz
```

3. Open another terminal and run publish_heatmap.py
```
python gazebo_wifi_plugin/publish_heatmap.py
```

### Follow Waypoints Simulation:
There is a hardcoded mowing-the-lawn example pattern available at `/trash_finder/config/mowing_lawn_waypts.yaml`

To get the AUV to follow the pattern:
```
roslaunch trash_finder oceans2020_demo.launch
roslaunch trash_finder send_waypoints_file.launch uuv_name:=rexrov2 interpolator:=linear 
```

### To run La Jolla environment:
To test if the La Jolla map works:

```
roslaunch lajolla_world lj.launch
```
