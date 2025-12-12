# turtlebot3_custom
ROS2 package for creating custom 3D worlds for turtlebot3 simulation and navigation

The world and model configurations are based on the turtlebot3_gazebo package from robotis
https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/main/turtlebot3_gazebo

The teleop_keyboard code is from turtlebot_teleop from robotis


## installation 
make sure to properly source the user (overlay) workspace

```
source ~/ros2_ws/install/local_setup.bash
```

add this line to `~/.bashrc` so it runs at the start of each new terminal
```
echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
```


clone this package into the user workspace source directory
```
cd ~/ros2_ws/src
git clone https://github.com/thillRobot/turtlebot3_custom.git
```

move to the root of the workspace and build the workspace
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

make sure to have a robot model selected, this can be added to ~/.bashrc also
this package has some customizations on 'waffle_pi' but the others are unmodified
```
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
```

test the simulator with robot in virtual TNTech AIEB robotics lab, room 143
```
killall -9 gzserver gzclient
ros2 launch turtlebot3_custom turtlebot3_custom.launch.py
```

drive the robot with the teleop_keyboard node
modified for faster travel in simulation 
``` 
ros2 run turtlebot3_custom teleop_keyboard.py
```

## Creating or modifiying a custom world 

to change the model in the custom world add mesh files to models/turtlebot3_custom/meshes

the files should be .stl or .dae 
the current file for the wallsshould be called `custom_walls_floor.stl` or the `turtlebot3_custom/model.sdf` needs to edited to have new filename

if the scale is correct, then it should work fine, if not adjust the values inside the scale tag
(model.sdf, lines 23-26), 
```
<mesh>
  <uri>model://turtlebot3_custom/meshes/custom_walls_floor.stl</uri>
  <scale>1.0 1.0 1.0</scale>
</mesh>
```

more elements can be added as separate models, each needs a `visual` and `collision` tag


## mapping the custom world 
use turtlebot cartographer to make a map of the custom world, this will take some time as the robot is slow and small

start the simulator in the custom world (shut down previous instances first)
```
killall -9 gzserver gzclient
ros2 launch turtlebot3_custom turtlebot3_custom.launch.py x_pose:=0 y_pose:=5
```

if the sim does not re-start correctly, run `killall ruby` and try agin,
also starting a new terminal can fix strange behaviour ( i think this is a gazebo ionic bug)

start the SLAM process 

```
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

drive the robot to make a map, drive slowly and in simple paths to make a clean map
if the robot slips or gets stuck, the map will be messy
```
ros2 run turtlebot3_custom teleop_keyboard.py
```

when finished, save the map before closing simulator or cartographer node
choose a new map name or overwrite `custom_map0`
```
cd ~/ros2_ws/src/turtlebot3_custom
ros2 run nav2_map_server map_saver_cli -f maps/custom_map0
```

## navigation in the custom world

start the simulator if it is not running, choose an initial robot pose
```
killall -9 gzserver gzclient
ros2 launch turtlebot3_custom turtlebot3_custom.launch.py x_pose:=0 y_pose:=5
```

test the map previously created with cartographer
```
cd ~/ros2_ws/src/turtlebot3_custom
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/custom_map0.yaml
``` 

## program a mission in the custom world 
the c++ node 'goal_publisher' can be used to programmatically send the robot to a goal

edit lines 38-41 in `src/goal_publisher.cpp` and build the workspace to adjust the goal, keep in mind navigation will fail if the goal is not acheiveable so this will need to be adjusted depending on the custom world and map

with the simulator and navigation running, run the goal_publihser node

```
ros2 run turtlebot3_custom goal_publisher
```

if everything is working correctly, the robot should plan a path and navigate to the goal
this node uses a open loop timing wait between goal, a better method is needed
 
