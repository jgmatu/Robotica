cmake_minimum_required(VERSION 2.8.3)
project(follow_line)

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
)

include_directories(
	${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}

)

add_definitions(${PCL_DEFINITIONS})

add_message_files(
  FILES
  Shift.msg
  Position.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}

)

add_executable(node_image_line
	src/node_image.cpp
)

add_executable(kobuki_correcting
	src/kobuki_correcting.cpp
)


target_link_libraries(node_image_line ${catkin_LIBRARIES})


target_link_libraries(kobuki_correcting ${catkin_LIBRARIES})
