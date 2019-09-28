#!/usr/bin/env python

import rospy
from subprocess import check_call, CalledProcessError
import sys
import os
import shutil

def print_help_message():
    print """
Usage: rosrun warehouse_ros_mongo mongo_wrapper_ros.py
Start the mongodb database server, configured using the following ROS parameters:

* parameters in parent namespace
  - warehouse_port: port used by db.  Defaults to 27017.
  - warehouse_host: doesn't directly affect server, but used by clients to know where the db is.

* parameters in parent namespace
  - db_path: where the db is stored on the filesystem.  Defaults to /tmp/db.
  - overwrite: whether to overwrite existing database if it exists.  Defaults to false.
"""

if '--help' in sys.argv:
    print_help_message()
    sys.exit()

rospy.init_node('mongodb')
path_param = rospy.get_param('~database_path' , None)
if path_param is None:
    path_param = rospy.get_param('~db_path' , '/tmp/db')
else:
    print 'Parameter "database_path" is deprecated. Please use "db_path" instead.'
dbpath = os.path.expanduser(path_param)
overwrite = rospy.get_param('~overwrite', False)

if '--repair' in sys.argv:
    rospy.loginfo("Repairing database")
    lock_file = '{0}/mongod.lock'.format(dbpath)
    if os.path.exists(lock_file):
        rospy.loginfo("  Removing lock file")
        os.remove(lock_file)
    check_call(['mongodb', 'mongod', '--repair', '--dbpath', dbpath.format(dbpath)])
    rospy.loginfo("  Successfully repaired.")
    sys.exit(0)

# The defaults here should match the ones used by each client library
port = rospy.get_param('warehouse_port', 27017)
host = rospy.get_param('warehouse_host', 'localhost')

if overwrite and os.path.exists(dbpath):
    shutil.rmtree(dbpath)
    rospy.loginfo('Removed existing db at %s', dbpath)

if not os.path.exists(dbpath):
    rospy.loginfo('{0} did not exist; creating it.'.format(dbpath))
    os.makedirs(dbpath)

rospy.loginfo('Starting mongodb with db location {0} listening on {2}:{1}'.\
              format(dbpath, port, host))

try:
    check_call("mongod --dbpath {} --port {}".format(dbpath, port).split())
except CalledProcessError as e:
    if e.returncode==12:
        rospy.loginfo("Ignoring mongod's non-standard return code of 12")
    else:
        rospy.logerr("Mongo process exited with error code {0}".format(e.returncode))
except OSError as e:
    rospy.logerr("Execution failed: %s", e)
