cmake_minimum_required(VERSION 2.8.3)
project(kf_factory)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  message_generation
  cv_bridge
  image_transport
  relative_nav  
  kb_autopilot
)

find_package(OpenCV 3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(keyframe_factory_node
  		src/keyframe_factory_node.cpp
)

target_link_libraries(keyframe_factory_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(keyframe_factory_node openfabmap_generate_messages_cpp relative_nav_generate_messages_cpp kb_autopilot_generate_messages_cpp)
