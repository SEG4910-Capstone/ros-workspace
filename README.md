# ros-workspace

## Building the workspace
```colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release```

## Sourcing the build in terminal
```source install/setup.bash``` (Assuming you are in ros-workspace) 
</br>

To not do this each time you open up a new terminal goto the 
```nano ~/.bashrc```, and add ```source {full path to your setup.bash file}```. Ex. ```source /home/ros-workspace/install/setup.bash```