cmake_minimum_required(VERSION 2.8.3)
project(warehouse_ros_mongo)

add_compile_options(-std=c++11)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  rostime
  std_msgs
  warehouse_ros
  class_loader
)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(OpenSSL REQUIRED)
find_package(MongoDB REQUIRED)

# At least gcc 7.4 has issues compiling against mongoclient with optimization > 1 enabled.
# In this case, Labeler symbols LT, GT, etc. are null, causing weird results.
if (CMAKE_COMPILER_IS_GNUCC)
  add_compile_options(-O1)
endif()

if(EXISTS "${MongoDB_INCLUDE_DIR}/mongo/version.h")
  add_definitions(-DWAREHOUSE_ROS_MONGO_HAVE_MONGO_VERSION_H)
endif()

set(MONGO_EXPORT)
if("${MongoDB_LIBRARIES}" MATCHES "\\.so$")
  set(MONGO_EXPORT MongoDB)
endif()

file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rostime std_msgs
  DEPENDS Boost ${MONGO_EXPORT}
)

if (NOT MongoDB_EXPOSE_MACROS)
  add_definitions(-DMONGO_EXPOSE_MACROS)
endif()

configure_file("include/${PROJECT_NAME}/config.h.in" "${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/config.h")

#catkin_lint: ignore_once external_directory  (${CATKIN_DEVEL_PREFIX}/include)
include_directories(${CATKIN_DEVEL_PREFIX}/include include ${catkin_INCLUDE_DIRS} ${MongoDB_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

set(warehouse_srcs
  src/query_results.cpp
  src/message_collection.cpp
  src/database_connection.cpp)

add_library(${PROJECT_NAME} SHARED ${warehouse_srcs})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${MongoDB_LIBRARIES} ${OPENSSL_LIBRARIES} ${Boost_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_executable(test_warehouse_ros_mongo_cpp test/test_warehouse_ros_mongo.cpp)
  target_link_libraries(test_warehouse_ros_mongo_cpp warehouse_ros_mongo ${GTEST_LIBRARIES})
  add_rostest(test/warehouse_ros_mongo.test)
endif()

install(PROGRAMS scripts/mongo_wrapper_ros.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
#catkin_lint: ignore_once external_file
install(FILES "${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/config.h"
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES mongo_database_connection_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
