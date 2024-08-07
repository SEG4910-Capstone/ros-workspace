# ros-workspace

## Installing the ros dependencies 
This will install the dependencies like navigation2, ros2_control, etc on to the system.<br>
```sudo rosdep init```<br>
```rosdep update```<br>
Navigate to the ros-workspace then <br>
```rosdep install --from-paths src --ignore-src -r -y```

## Building the workspace
```colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release```

## Sourcing the build in terminal
```source install/setup.bash``` (Assuming you are in ros-workspace) 
</br>

To not do this each time you open up a new terminal go to the 

```nano ~/.bashrc```, and add ```source {full path to your setup.bash file}```. Ex. ```source /home/ros-workspace/install/setup.bash```

## Mapping
To launch the simulation there are 3 different launch commands
<br>
```ros2 launch snowplow launch_sim_gps.launch.py```
<br>
This command launches the virtual environment with the simulated physical robot and sensors.
<br>
```ros2 launch snowplow slam.launch.py use_sim_time:=true params_file:=[location to mapping slam config]``` 
<br>
This command launches the SLAM toolbox in mapping mode where you can save the map after you navigate around the area. The params_file parameter is optional.
<br>
```ros2 launch snowplow navigation_launch.py use_sim_time:=true```
<br>
This command launches the nav2 stack which has all the plugins for waypoint following, behavior tree, path planning, etc.

## Localization
After generating a map of the surrounding, AMCL can be used to localize the robot within a given area. For this there are 3 launch commands to run.
<br>
```ros2 launch snowplow launch_sim_gps.launch.py```
<br>
This command launches the virtual environment with the simulated physical robot and sensors. 
<br>
```ros2 launch snowplow localization_launch.py use_sim_time:=true map:=[the yaml file generated from SLAM mapping] params_file:=[location to nav2 config```
<br>
This command launches the map server and AMCL. The params_file parameter is optional again as it is defaulted to a preset config file. 
<br>
```ros2 launch snowplow navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true default_bt_xml_filename:=[path to bt tree file]```
<br>
This command launches the nav2 stack which has all the plugins for waypoint following, behavior tree, path planning, etc. defaukt_bt_xml_filename parameter is optional and defaults to an existing config if nothing is passed.
