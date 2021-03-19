# - Find MONGODB; NOTE: this is specific to warehouse_ros!
#
# Find the MONGODB includes and client library
# This module defines
#  MONGODB_INCLUDE_DIR, where to find mongo/client/dbclient.h
#  MONGODB_LIBRARIES, the libraries needed to use MONGODB.
#  MONGODB_FOUND, If false, do not try to use MONGODB.
#  MONGODB_EXPOSE_MACROS_FLAG, If true, warehouse_ros_mongo should use
#    '#define MONGO_EXPOSE_MACROS'

set(MONGODB_EXPOSE_MACROS_FLAG "NO")

set(MONGODB_PossibleIncludePaths
  /usr/include/
  /usr/local/include/
  /usr/include/mongo/
  /usr/local/include/mongo/
  /opt/mongo/include/
  $ENV{ProgramFiles}/Mongo/*/include
  $ENV{SystemDrive}/Mongo/*/include
)
find_path(MONGODB_INCLUDE_DIR mongo/client/dbclient.h
  ${MONGODB_PossibleIncludePaths})

if(MONGODB_INCLUDE_DIR)
  find_path(MONGODB_dbclientinterface_Path mongo/client/dbclientinterface.h
    ${MONGODB_PossibleIncludePaths})
  if(MONGODB_dbclientinterface_Path)
    set(MONGODB_EXPOSE_MACROS_FLAG "YES")
  endif()
endif()

if(WIN32)
  find_library(MONGODB_LIBRARIES NAMES mongoclient
    PATHS
    $ENV{ProgramFiles}/Mongo/*/lib
    $ENV{SystemDrive}/Mongo/*/lib
  )
else()
  find_library(MONGODB_LIBRARIES NAMES mongoclient
    PATHS
    /usr/lib
    /usr/lib64
    /usr/lib/mongo
    /usr/lib64/mongo
    /usr/local/lib
    /usr/local/lib64
    /usr/local/lib/mongo
    /usr/local/lib64/mongo
    /opt/mongo/lib
    /opt/mongo/lib64
  )
endif()

if(MONGODB_INCLUDE_DIR AND MONGODB_LIBRARIES)
  set(MONGODB_FOUND TRUE)
  message(STATUS "Found MONGODB: ${MONGODB_INCLUDE_DIR}, ${MONGODB_LIBRARIES}")
  message(STATUS "MONGODB using new interface: ${MONGODB_EXPOSE_MACROS_FLAG}")
else()
  set(MONGODB_FOUND FALSE)
  if(MONGODB_FIND_REQUIRED)
    message(FATAL_ERROR "MONGODB not found.")
  else()
    message(STATUS "MONGODB not found.")
  endif()
endif()

mark_as_advanced(MONGODB_INCLUDE_DIR MONGODB_LIBRARIES MONGODB_EXPOSE_MACROS_FLAG)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MONGODB DEFAULT_MSG
  MONGODB_LIBRARIES MONGODB_INCLUDE_DIR)
