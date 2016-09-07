## FindMKL
## Mantainer: Joao Reis <joaocgreis@gmail.com>

IF (MKL_INCLUDE_DIRS AND MKL_LIBRARIES)
   # in cache already - wont print a message
   SET(MKL_FIND_QUIETLY TRUE)
ENDIF ()

########################################################################
##  general find

IF(DEFINED ENV{MKL_DIR})
  SET(MKL_DIR $ENV{MKL_DIR})
ELSE()
  # use default
  SET(MKL_DIR "/opt/intel/mklsocrob")
ENDIF()

IF(EXISTS ${MKL_DIR})
  SET(MKL_INCLUDE_DIRS "${MKL_DIR}/include")

  IF(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    SET(MKL_LIBRARIES "-L${MKL_DIR}/lib/em64t -lmkl_intel_lp64 -lmkl_intel_thread -lmkl_core" )
  ELSE()
    SET(MKL_LIBRARIES "-L${MKL_DIR}/lib/32 -lmkl_intel -lmkl_intel_thread -lmkl_core" )
  ENDIF()

  SET(IPP_LIBRARIES "${IPP_LIBRARIES} -lguide")

ENDIF()

########################################################################
## finished - now just set up flags

INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(MKL DEFAULT_MSG MKL_LIBRARIES MKL_INCLUDE_DIRS)

MARK_AS_ADVANCED(MKL_INCLUDE_DIRS MKL_LIBRARIES)
