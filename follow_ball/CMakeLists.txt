cmake_minimum_required(VERSION 2.8.3)
project(follow_ball)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  image_transport
  message_generation
  actionlib
  actionlib_msgs
  #csuro_tools  
)

add_message_files(
   FILES
   Point.msg
   Position.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)

include_directories(
	${PCL_INCLUDE_DIRS} 
)

link_directories(
	${PCL_LIBRARY_DIRS} 
)

add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES csuro_comms_cpp
  CATKIN_DEPENDS roscpp std_msgs message_runtime actionlib_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(node_image_ball
	src/node_image_ball.cpp
)


target_link_libraries(node_image_ball ${catkin_LIBRARIES})

add_executable(go_ball
	src/go_ball.cpp
	src/PIDController.cpp
)

target_link_libraries(go_ball ${catkin_LIBRARIES})





