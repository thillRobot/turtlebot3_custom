# turtlebot3_custom
ROS2 package for creating custom 3D worlds for turtlebot3 simulation and navigation

The world and model configuration is based on the turtlebot3_gazebo package from robotis
https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/main/turtlebot3_gazebo

# installation 
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

set the model directory with the GZ yayay environment var
```
export GZ_SIM_RESOURCE_PATH=~/ros2_ws/src/turtlebot3_custom/models/
```

test the robot in TNTech Brownhall third floor
```
ros2 launch turtlebot3_custom turtlebot3_brown3.launch.py
```


# Creating a custom world 
