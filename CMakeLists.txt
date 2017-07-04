cmake_minimum_required(VERSION 2.8.3)

project(pfuclt_omni_dataset)

#set(CMAKE_BUILD_TYPE Debug)

OPTION(USE_OpenMP "Use OpenMP" ON)
IF(USE_OpenMP)
  FIND_PACKAGE(OpenMP)
  IF(OPENMP_FOUND)
    message(STATUS "OPENMP FOUND")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    SET(OpenMP_LIBS gomp)
  ENDIF()
ENDIF()

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

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} include)

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

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${Eigen3_INCLUDE_DIRS})

find_package(Boost COMPONENTS filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

set(HEADER_FILES include/pfuclt_omni_dataset/pfuclt_aux.h include/pfuclt_omni_dataset/pfuclt_omni_dataset.h include/pfuclt_omni_dataset/pfuclt_particles.h)
set(SOURCE_FILES src/pfuclt_omni_dataset.cpp src/pfuclt_aux.cpp src/pfuclt_particles.cpp)

add_executable(pfuclt_omni_dataset ${HEADER_FILES} ${SOURCE_FILES})
target_compile_options(pfuclt_omni_dataset PRIVATE ${OpenMP_FLAGS})
add_dependencies(pfuclt_omni_dataset pfuclt_omni_dataset_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(pfuclt_omni_dataset ${catkin_LIBRARIES} ${rosbag_LIBRARIES} ${Eigen3_LIBRARIES} ${Boost_LIBRARIES} ${read_omni_dataset_LIBRARIES} minicsv ${OpenMP_LIBS})
