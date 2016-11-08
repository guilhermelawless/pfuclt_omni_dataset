cmake_minimum_required(VERSION 2.8.3)

project(pfuclt_omni_dataset)

#set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules")
find_package(
  cmake_modules REQUIRED
  catkin REQUIRED dynamic_reconfigure
)

#external includes
INCLUDE_DIRECTORIES(include)

#build library for minicsv
add_library(minicsv src/minicsv.cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs 
  tf 
  geometry_msgs     
  nav_msgs
  message_generation
  tf2
  tf2_ros
  dynamic_reconfigure
)

FIND_PACKAGE(read_omni_dataset REQUIRED)
INCLUDE_DIRECTORIES(${read_omni_dataset_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
  config/Dynamic.cfg
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
  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy
#CATKIN_DEPENDS roscpp rospy message_runtime read_omni_dataset
#  DEPENDS system_lib
)

FIND_PACKAGE(Eigen REQUIRED)
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})

find_package(Boost COMPONENTS filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

set(SOURCE_FILES src/pfuclt_omni_dataset.cpp src/pfuclt_aux.cpp src/pfuclt_particles.cpp)

add_executable(pfuclt_omni_dataset ${SOURCE_FILES})
add_dependencies(pfuclt_omni_dataset pfuclt_omni_dataset_generate_messages_cpp)
target_link_libraries(pfuclt_omni_dataset ${catkin_LIBRARIES} ${rosbag_LIBRARIES} ${read_omni_dataset_LIBRARIES} minicsv)

