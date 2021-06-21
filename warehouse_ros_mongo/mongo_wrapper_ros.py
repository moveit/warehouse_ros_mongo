#!/usr/bin/env python3

# Copyright 2008 Willow Garage, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from __future__ import print_function
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from subprocess import check_call, CalledProcessError
import sys
import os
import shutil


default_db_path = "/tmp/db"


def print_help_message():
    print(
        """
Usage: rosrun warehouse_ros_mongo mongo_wrapper_ros.py
Start the mongodb database server, configured using the following ROS parameters:

* parameters in parent namespace
  - warehouse_port: port used by db.  Defaults to 27017.
  - warehouse_host: doesn't directly affect server, but used by clients to know where the db is.

* parameters in parent namespace
  - db_path: where the db is stored on the filesystem.  Defaults to {0}.
  - overwrite: whether to overwrite existing database if it exists.  Defaults to false.
""".format(
            default_db_path
        )
    )


if "--help" in sys.argv:
    print_help_message()
    sys.exit()

rclpy.init()
node = Node(
    "mongodb",
    allow_undeclared_parameters=True,
    automatically_declare_parameters_from_overrides=True,
)

path_param = node.get_parameter_or(
    "database_path", Parameter("", Parameter.Type.STRING, "None")
)
if path_param.value == "None":
    path_param = node.get_parameter_or(
        "db_path", Parameter("", Parameter.Type.STRING, default_db_path)
    )
else:
    print('Parameter "database_path" is deprecated. Please use "db_path" instead.')
dbpath = os.path.expanduser(path_param.value)
overwrite = node.get_parameter_or("overwrite", False)

if "--repair" in sys.argv:
    node.get_logger().info("Repairing database")
    lock_file = "{0}/mongod.lock".format(dbpath)
    if os.path.exists(lock_file):
        node.get_logger().info("  Removing lock file")
        os.remove(lock_file)
    check_call(["mongodb", "mongod", "--repair", "--dbpath", dbpath])
    node.get_logger().info("  Successfully repaired.")
    sys.exit(0)

# The defaults here should match the ones used by each client library
port = node.get_parameter_or(
    "warehouse_port", Parameter("", Parameter.Type.INTEGER, 33829)
)
host = node.get_parameter_or(
    "warehouse_host", Parameter("", Parameter.Type.STRING, "localhost")
)

if overwrite and os.path.exists(dbpath):
    node.get_logger().info("Removed existing db at {}".format(dbpath))
    shutil.rmtree(dbpath)

while not os.path.exists(dbpath):
    node.get_logger().info("{0} does not exist; creating it.".format(dbpath))
    try:
        os.makedirs(dbpath)
        break
    except OSError as e:
        if dbpath != default_db_path:
            node.get_logger().error(
                "{0}. Trying {1} instead.".format(str(e), default_db_path)
            )
            dbpath = default_db_path
        else:
            node.get_logger().fatal(e)
            raise e

node.get_logger().info(
    "Starting mongodb with db location {0} listening on {2}:{1}".format(
        dbpath, port.value, host.value
    )
)

try:
    check_call("mongod --dbpath {} --port {}".format(dbpath, port.value).split())
except CalledProcessError as e:
    if e.returncode == 12:
        node.get_logger().info("Ignoring mongod's non-standard return code of 12")
    else:
        node.get_logger().error(
            "Mongo process exited with error code {0}".format(e.returncode)
        )
except OSError as e:
    node.get_logger().error("Execution failed: {}".format(e))
