## FindIPP
## Mantainer: Joao Reis <joaocgreis@gmail.com>

IF (IPP_INCLUDE_DIRS AND IPP_LIBRARIES)
   # in cache already - wont print a message
   SET(IPP_FIND_QUIETLY TRUE)
ENDIF ()

########################################################################
##  general find

IF(DEFINED ENV{IPP_DIR})
  SET(IPP_DIR $ENV{IPP_DIR})
ELSE()
  # use default
  SET(IPP_DIR "/opt/intel/ipp")
ENDIF()

IF(EXISTS ${IPP_DIR})
  SET(IPP_INCLUDE_DIRS "${IPP_DIR}/include")

  IF( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" )                                                                                         
    SET(IPP_LIBRARIES "-L${IPP_DIR}/lib/intel64 -lippi -lippcore -lippcc -lipps  -lippm -lippvm -lippcv" )                                                                                                                
  ELSE( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" )                                                                                       
    SET(IPP_LIBRARIES "-L${IPP_DIR}/lib/intel64 -lippi -lippcore -lippcc -lipps -lippm -lippvm -lippcv" )         
  ENDIF( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" )                                                                                      

  SET(IPP_LIBRARIES "${IPP_LIBRARIES} -liomp5 -lpthread")

ENDIF()

########################################################################
## finished - now just set up flags

INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(IPP DEFAULT_MSG IPP_LIBRARIES IPP_INCLUDE_DIRS)

MARK_AS_ADVANCED(IPP_INCLUDE_DIRS IPP_LIBRARIES)
