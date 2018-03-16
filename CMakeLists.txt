cmake_minimum_required(VERSION 2.8.3)
project(evolutive_map)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# FIND PACKAGES
find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    tf
    tf2
)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS roscpp gazebo_ros
)

add_executable(life_cycle src/life_cycle/life_cycle.cpp src/life_cycle/Container.cpp src/life_cycle/detect_containers.cpp)
add_executable(automove src/automove/automove.cpp)
add_executable(odom_noise src/odom_noise/odom_noise_node.cpp)
add_executable(calibrate_odom_noise src/odom_noise/calibrate_odom_noise_node.cpp)
add_executable(laser_noise src/laser_noise/laser_noise_node.cpp)

target_link_libraries(life_cycle ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(automove ${catkin_LIBRARIES})
target_link_libraries(odom_noise ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(calibrate_odom_noise ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(laser_noise ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
