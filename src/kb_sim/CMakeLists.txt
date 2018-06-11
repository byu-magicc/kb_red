cmake_minimum_required(VERSION 2.8.3)
project(kb_sim)
add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  image_transport
  tf
  gazebo_plugins
  # message_generation
)
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES drcsim_gazebo_plugins drcsim_gazebo_ros_plugins
    DEPENDS
    roscpp
    gazebo_ros
    gazebo_plugins
)

# Depend on system install of Gazebo and Boost
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
  ${PROJECT_SOURCE_DIR}/include  #hm, is this ok?
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# compile and install gazebo plugins
add_library(drcsim_gazebo_plugins SHARED src/DRCVehiclePlugin.cc)
target_link_libraries(drcsim_gazebo_plugins ${GAZEBO_LIBRARIES})
install (TARGETS drcsim_gazebo_plugins DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/drcsim_gazebo_plugins/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(drcsim_gazebo_ros_plugins src/DRCVehicleROSPlugin.cpp)
target_link_libraries(drcsim_gazebo_ros_plugins ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} drcsim_gazebo_plugins)
add_dependencies(drcsim_gazebo_ros_plugins drcsim_gazebo_plugins)
