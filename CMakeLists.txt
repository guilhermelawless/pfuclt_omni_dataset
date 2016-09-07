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


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
# 	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# elseif(COMPILER_SUPPORTS_CXX0X)
# 	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
# else()
#         message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()

FIND_PACKAGE(Eigen REQUIRED)
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})

find_package(Boost COMPONENTS filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# IPP
## FindIPP
## Mantainer: Joao Reis <joaocgreis@gmail.com>

IF (IPP_INCLUDE_DIRS AND IPP_LIBRARIES)
   # in cache already - wont print a message
   SET(IPP_FIND_QUIETLY TRUE)
ENDIF ()

########################################################################
##  general find

# IF(DEFINED ENV{IPP_DIR})
#   SET(IPP_DIR $ENV{IPP_DIR})
# ELSE()
  # use default
  SET(IPP_DIR "/opt/intel/composer_xe_2011_sp1.8.273/ipp")
# ENDIF()

IF(EXISTS ${IPP_DIR})
  SET(IPP_INCLUDE_DIRS "${IPP_DIR}/include")

  IF( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" )                                                                                         
    SET(IPP_LIBRARIES "-L${IPP_DIR}/lib/intel64 -lippi -lippcore -lippcc -lipps  -lippm -lippvm -lippcv" )                                                                                                                
  ELSE( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" )                                                                                       
    SET(IPP_LIBRARIES "-L${IPP_DIR}/lib/intel64 -lippi -lippcore -lippcc -lipps -lippm -lippvm -lippcv" )         
  ENDIF( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" )                                                                                      

#   SET(IPP_LIBRARIES "${IPP_LIBRARIES} -liomp5 -lpthread")
SET(IPP_LIBRARIES "${IPP_LIBRARIES} -liomp5 -lpthread")

ENDIF()

########################################################################
## finished - now just set up flags

INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(IPP DEFAULT_MSG IPP_LIBRARIES IPP_INCLUDE_DIRS)

MARK_AS_ADVANCED(IPP_INCLUDE_DIRS IPP_LIBRARIES)


add_executable(pfuclt_omni_dataset src/pfuclt_omni_dataset.cpp)
add_dependencies(pfuclt_omni_dataset pfuclt_omni_dataset_generate_messages_cpp)
target_link_libraries(pfuclt_omni_dataset ${catkin_LIBRARIES} ${rosbag_LIBRARIES} ${IPP_LIBRARIES})

