cmake_minimum_required(VERSION 2.8.3)
project(pfuclt_omni_dataset)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules")
find_package(cmake_modules REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs 
  tf 
  geometry_msgs     
  nav_msgs
  message_generation
)

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  particle.msg
  particles.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy
CATKIN_DEPENDS roscpp rospy message_runtime
#  DEPENDS system_lib
)

FIND_PACKAGE(Eigen REQUIRED)
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})

find_package(Boost COMPONENTS filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(pfuclt_omni_dataset src/pfuclt_omni_dataset.cpp)
add_dependencies(pfuclt_omni_dataset pfuclt_omni_dataset_generate_messages_cpp)
target_link_libraries(pfuclt_omni_dataset ${catkin_LIBRARIES} ${rosbag_LIBRARIES})

