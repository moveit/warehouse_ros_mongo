cmake_minimum_required(VERSION 3.10.2)
project(warehouse_ros_mongo)

add_compile_options(-std=c++14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(warehouse_ros REQUIRED)
find_package(class_loader REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(OpenSSL REQUIRED)
find_package(MONGODB REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

# At least gcc 7.4 has issues compiling against mongoclient with optimization > 1 enabled.
# In this case, Labeler symbols LT, GT, etc. are null, causing weird results.
if(CMAKE_COMPILER_IS_GNUCC)
  add_compile_options(-O1)
endif()

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

configure_file("include/${PROJECT_NAME}/config.h.in" "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/config.h")

include_directories(${CMAKE_BINARY_DIR}/include include ${MONGODB_INCLUDE_DIR})

set(warehouse_srcs
  src/query_results.cpp
  src/message_collection.cpp
  src/database_connection.cpp)

add_library(${PROJECT_NAME} SHARED ${warehouse_srcs})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  warehouse_ros
  class_loader
  Boost
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
endif()

install(PROGRAMS ${PROJECT_NAME}/mongo_wrapper_ros.py DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include
        FILES_MATCHING PATTERN "*.h")
#catkin_lint: ignore_once external_file
install(FILES "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/config.h"
        DESTINATION include/${PROJECT_NAME})

install(FILES test/warehouse_ros_mongo.launch.py
        DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(warehouse_ros)
ament_export_dependencies(class_loader)
ament_export_dependencies(Boost)
ament_export_dependencies(OpenSSL)
ament_export_dependencies(MONGODB)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclpy)

pluginlib_export_plugin_description_file(warehouse_ros mongo_database_connection_plugin_description.xml)

ament_package()
