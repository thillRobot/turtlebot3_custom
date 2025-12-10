cmake_minimum_required(VERSION 3.20)
project(turtlebot3_custom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(ament_cmake_python REQUIRED) # Important for Python integration
find_package(rclpy REQUIRED) # Or other Python dependencies

file(MAKE_DIRECTORY "include")
file(MAKE_DIRECTORY "rviz")
file(MAKE_DIRECTORY "lib")


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${GZ_SIM_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${gz_sim_vendor_INCLUDE_DIRS}
  ${GZ_SIM_INCLUDE_DIRS}
)

set(EXEC_NAME "turtlebot3_drive")

add_executable(${EXEC_NAME} src/turtlebot3_drive.cpp)
target_link_libraries(${EXEC_NAME}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
)

set(OBSTACLE_LIBS
  obstacles
  obstacle1
  obstacle2
)

foreach(lib ${OBSTACLE_LIBS})
  add_library(${lib} SHARED src/${lib}.cpp)
  target_link_libraries(${lib}
    PUBLIC
    gz-sim::gz-sim
    gz-math::gz-math
  )
endforeach()


add_executable(goal_publisher src/goal_publisher.cpp)
target_link_libraries(goal_publisher PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS}
	                                                   ${geometry_msgs_TARGETS})
# add name of each executable to list of targets
install(TARGETS 
	goal_publisher	
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch models params rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY include/
  DESTINATION include/
)

ament_python_install_package(${PROJECT_NAME}) # This handles the Python package installation

# Install Python scripts as executables
install(PROGRAMS
  ${PROJECT_NAME}/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(
  gazebo_ros_pkgs
  geometry_msgs
  nav_msgs
  rclcpp
  rclpy
  sensor_msgs
  tf2
  goal_publisher	
)

ament_package()
