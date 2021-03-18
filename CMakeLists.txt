cmake_minimum_required(VERSION 3.10.2)
project(warehouse_ros_mongo)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set compile options 
set(PROJECT_COMPILE_OPTIONS 
  -Wall 
  -Wextra 
  -Wpedantic
)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(warehouse_ros REQUIRED)
find_package(class_loader REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(MONGODB REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Find package for Boost component dependencies
include(ConfigExtras.cmake)

include(CheckIncludeFileCXX)
check_include_file_cxx("mongo/version.h" HAVE_MONGO_VERSION_H)
if(HAVE_MONGO_VERSION_H)
  add_definitions(-DWAREHOUSE_ROS_MONGO_HAVE_MONGO_VERSION_H)
endif()

set(MONGO_EXPORT)
if("${MONGODB_LIBRARIES}" MATCHES "\\.so$")
  set(MONGO_EXPORT MONGODB)
endif()

if(NOT MONGODB_EXPOSE_MACROS_FLAG)
  add_definitions(-DMONGO_EXPOSE_MACROS)
endif()

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

# At least gcc 7.4 has issues compiling against mongoclient with optimization > 1 enabled.
# In this case, Labeler symbols LT, GT, etc. are null, causing weird results.
if(CMAKE_COMPILER_IS_GNUCC)
  add_compile_options(-O1)
endif()

configure_file("include/${PROJECT_NAME}/config.h.in" "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/config.h")

include_directories(${CMAKE_BINARY_DIR}/include include ${MONGODB_INCLUDE_DIR})

set(warehouse_srcs
  src/query_results.cpp
  src/message_collection.cpp
  src/database_connection.cpp)

add_library(${PROJECT_NAME} SHARED ${warehouse_srcs})
target_compile_options(${PROJECT_NAME} PRIVATE ${PROJECT_COMPILE_OPTIONS})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  warehouse_ros
  class_loader
  OpenSSL
)
target_link_libraries(${PROJECT_NAME} ${MONGODB_LIBRARIES})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/warehouse_ros_mongo.launch.py)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_warehouse_ros_mongo_cpp test/test_warehouse_ros_mongo.cpp)
  target_link_libraries(test_warehouse_ros_mongo_cpp warehouse_ros_mongo ${GTEST_LIBRARIES} ${OPENSSL_CRYPTO_LIBRARY})

  install(TARGETS test_warehouse_ros_mongo_cpp
    DESTINATION lib/${PROJECT_NAME}
  )

  # ament_lint
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS ${PROJECT_NAME}/mongo_wrapper_ros.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include
        FILES_MATCHING PATTERN "*.h")
#catkin_lint: ignore_once external_file
install(FILES "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/config.h"
        DESTINATION include/${PROJECT_NAME})

install(FILES test/warehouse_ros_mongo.launch.py
        DESTINATION share/${PROJECT_NAME})

install(
  FILES cmake/FindMONGODB.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake
  rclcpp
  std_msgs
  warehouse_ros
  class_loader
  OpenSSL
  ament_cmake_python
  rclpy
  MONGODB
)

pluginlib_export_plugin_description_file(warehouse_ros mongo_database_connection_plugin_description.xml)

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
