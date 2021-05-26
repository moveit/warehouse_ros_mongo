# Warehouse ROS Mongo Interface

Code for persisting ROS message data using MongoDB.  Contains C++ and Python libraries to serialize ROS data to MongoDB, as well as some handy scripts to record data from the command line.  Based on code split out of warehouse_ros.

## GitHub Actions - Continuous Integration

[![Formatting (pre-commit)](https://github.com/ros-planning/warehouse_ros_mongo/actions/workflows/format.yml/badge.svg?branch=ros2)](https://github.com/ros-planning/warehouse_ros_mongo/actions/workflows/format.yml?query=branch%3Aros2) [![Build And Test](https://github.com/ros-planning/warehouse_ros_mongo/actions/workflows/industrial_ci_action.yml/badge.svg?branch=ros2)](https://github.com/ros-planning/warehouse_ros_mongo/actions/workflows/industrial_ci_action.yml?query=branch%3Aros2) [![codecov](https://codecov.io/gh/ros-planning/warehouse_ros_mongo/branch/ros2/graph/badge.svg?token=W7uHKcY0ly)](https://codecov.io/gh/ros-planning/warehouse_ros_mongo)

[![Code Coverage Grid](https://codecov.io/gh/ros-planning/warehouse_ros_mongo/branch/ros2/graphs/tree.svg)](https://codecov.io/gh/ros-planning/warehouse_ros_mongo/branch/ros2/graphs/tree.svg)

## Building from source

### ROS Jade  / Kinetic

In order to build from source you'll need to install the [mongo c++ drivers](https://github.com/mongodb/mongo-cxx-driver/wiki/Download-and-Compile-the-Legacy-Driver)

First get the driver:
```
git clone -b 26compat https://github.com/mongodb/mongo-cxx-driver.git
```

Then compile using scons:
```
sudo apt-get install scons
cd mongo-cxx-driver
sudo scons --prefix=/usr/local/ --full --use-system-boost --disable-warnings-as-errors
```

You should now be able to compile the packages using catkin.
