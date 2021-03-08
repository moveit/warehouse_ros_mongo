# Extras module needed for dependencies of warehouse_ros_mongo

# This is needed so dependencies can find_package on MONGODB
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")

# Boost component library dependencies have to be exported like this
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

