# oceans_2020 abstract demo

### Installation/Setting up Environment
Shenge's wifi environment requries Kinetic and Gazebo 9. You can just use the docker image called ros-kinetic-gazebo9

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

From https://github.com/CogRob/cogrob_ros, clone the whole repo and link the folders to src:
```
(after you clone CogRob outside of Oceans_2020)
cd /oceans_2020/src
ln -s ~/cogrob_ros/fetch_gazebo_wifi/ .
ln -s ~/cogrob_ros/gazebo_wifi_plugin/ .
```

You should have the following structure:
```
catkin_ws/
    src/
        fetch_gazebo_wifi/
        gazebo_wifi_plugin/
        rexrov2/
        trash_finder/
        trash_worlds/
        uuv_simulation_evaluation/
        uuv_simulator/
```

Then in your ~/catkin_ws do:
```
catkin build
```

### Adaptive Information Sampling

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

**TODO:** Make waypoint filename a parameter from send_waypoints_file.launch instead of hardcode.

### Current work:
<!--
Currently, the only files of interest are at:
```
/catkin_ws/src/uuv_simulator/uuv_gazebo_worlds/launch/trash_lake.launch
/catkin_ws/src/uuv_simulator/uuv_gazebo_worlds/world/trash_lake.world
/catkin_ws/src/rexrov2/rexrov2_gazebo/launch/oceans2020_demo.launch
/catkin_ws/src/gazebo_wifi_plugin/
/catkin_ws/src/fetch_gazebo_wifi/
```
-->

I can't get the heatmap from the wifi boxes in gazebo here. To replicate my steps:

```
cd /catkin_ws/src/
roslaunch rexrov2_gazebo oceans2020_demo.launch
```
The wifi boxes are on one side of the lake. Zoom allll the way out and then look for a white square on one of the flat surfaces around the edge. That is where the boxes should be. I wanted to make sure the problem wasn't that the wifi signal was underwater.

I have a feeling that the error is something small. The following error message is from the roslaunch output. Maybe we just have to move the src code from fetch_gazebo_wifi so it can find the sensor?
```
Error:
[Wrn] [msgs.cc:1852] Conversion of sensor type[wireless_transmitter] not supported.
[Wrn] [msgs.cc:1852] Conversion of sensor type[wireless_receiver] not supported.
```
