# turtlebot3_custom
ROS2 package for creating custom 3D worlds for turtlebot3 simulation and navigation

The world and model configurations are based on the turtlebot3_gazebo package from robotis
https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/main/turtlebot3_gazebo

The teleop_keyboard code is from turtlebot_teleop from robotis

## re-install gazebo sim from ROS 
first remove previous installations of gazebo
```
  sudo apt remove gz-ionic && sudo apt autoremove
```

Install Gazebo Ionic (recommended version for ROS Kilted) from ros binaries

```
  sudo apt-get update
  sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```
Test the simulation program installed correctly
```
  gz sim
```



## installation 
clone this package into a working ROS2 workspace source directory
```
cd ~/ros2_ws/src
git clone https://github.com/thillRobot/turtlebot3_custom.git
```

build the workspace
```
cd ~/ros2_ws
colcon build
```

set the model directory with the new GZ environment var
put this in ~/.bashrc so it runs each time
```
echo "export GZ_SIM_RESOURCE_PATH=~/ros2_ws/src/turtlebot3_custom/models/" >> ~/.bashrc
source ~/.bashrc
```

test the robot in TNTech AIEB robotics lab, room 143
```
ros2 launch turtlebot3_custom turtlebot3_custom.launch.py
```

drive the robot with the teleop_keyboard node
modified for faster travel in simulation 
``` 
ros2 run turtlebot3_custom teleop_keyboard.py
```

## Creating or modifiying a custom world 

to change the model in the custom world you can add mesh files to models/turtlebot3_custom/meshes
the files should be .stl or .dae 
the file for the walls  should be called `custom_wall.stl` or the turtlebot3_custom/model.sdf needs to have new filename
if the scale is correct, then it should work fine, if not adjust the values inside the scale tag
(model.sdf, lines 23-26)
```
<mesh>
  <uri>model://turtlebot3_custom/meshes/custom_walls.stl</uri>
  <scale>1.0 1.0 1.0</scale>
</mesh>
```

more elements can be added as separate models, each needs a `visual` and `collision` tag


## mapping the custom world 
use turtlebot cartographer to make a map of the custom world, this will take some time as the robot is slow and small

start the simulator in the custom world
```
ros2 launch turtlebot3_custom turtlebot3_custom.launch.py
```

start the SLAM process 

```
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

drive the robot to make a map
```
ros2 run turtlebot3_custom teleop_keyboard.py
```

when finished, save the map before closing simulator or cartographer node
```
  ros2 run nav2_map_server map_saver_cli -f maps/custom_map0
```

## navigation in the custom world

start the simulator if it is not running
```
ros2 launch turtlebot3_custom turtlebot3_custom.launch.py
```

test the map created with nav2 
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/custom_map0.yaml
``` 




