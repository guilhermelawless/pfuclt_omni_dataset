## FindACE
## Mantainer: Joao Reis <joaocgreis@gmail.com>

IF (ACE_INCLUDE_DIRS AND ACE_LIBRARIES)
   # in cache already - wont print a message
   SET(ACE_FIND_QUIETLY TRUE)
ENDIF ()

########################################################################
## check pkg-config for ace information, if available

IF(PKG_CONFIG_FOUND)
  PKG_CHECK_MODULES (ACE ACE)
  SET (ACE_INCLUDE_DIRS ${ACE_INCLUDEDIR})
ENDIF()

########################################################################
##  general find

FIND_PATH( ACE_INCLUDE_DIRS ace/ACE.h
           ${CMAKE_SOURCE_DIR}/../ACE_wrappers/
           /usr/include
           /usr/local/include
           $ENV{ACE_ROOT}
           $ENV{ACE_ROOT}/include
           DOC "directory containing ace/*.h for ACE library" )

FIND_LIBRARY( ACE_LIBRARIES
              NAMES ACE ace
              PATHS
              ${CMAKE_SOURCE_DIR}/../ACE_wrappers/lib/
              /usr/lib
              /usr/local/lib
              $ENV{ACE_ROOT}/lib
              $ENV{ACE_ROOT}
              DOC "ACE library file" )

########################################################################
## finished - now just set up flags

INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(ACE DEFAULT_MSG ACE_LIBRARIES ACE_INCLUDE_DIRS)

MARK_AS_ADVANCED(ACE_INCLUDE_DIRS ACE_LIBRARIES)
