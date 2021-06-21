// Copyright 2008 Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage, Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <geometry_msgs/msg/pose.hpp>
#include <warehouse_ros/message_with_metadata.h>
#include <warehouse_ros_mongo/metadata.h>
#include <cstring>

const double TOL = 1e-3;
using std::ostream;

namespace geometry_msgs
{
inline bool operator==(const msg::Pose& p1, const msg::Pose& p2)
{
  const msg::Point& pos1 = p1.position;
  const msg::Point& pos2 = p2.position;
  const msg::Quaternion& q1 = p1.orientation;
  const msg::Quaternion& q2 = p2.orientation;
  return (pos1.x == pos2.x) && (pos1.y == pos2.y) && (pos1.z == pos2.z) && (q1.x == q2.x) && (q1.y == q2.y) &&
         (q1.z == q2.z) && (q1.w == q2.w);
}
}  // namespace geometry_msgs

inline warehouse_ros_mongo::MongoMetadata& downcastMetadata(warehouse_ros::Metadata::ConstPtr metadata)
{
  return *(const_cast<warehouse_ros_mongo::MongoMetadata*>(
      static_cast<const warehouse_ros_mongo::MongoMetadata*>(metadata.get())));
}

inline warehouse_ros_mongo::MongoQuery& downcastQuery(warehouse_ros::Query::ConstPtr query)
{
  return *(
      const_cast<warehouse_ros_mongo::MongoQuery*>(static_cast<const warehouse_ros_mongo::MongoQuery*>(query.get())));
}

template <class T>
ostream& operator<<(ostream& str, const warehouse_ros::MessageWithMetadata<T>& s)
{
  // TODO: revert when message stream is enabled
  // const T& msg = s;
  // str << "Message: " << msg;
  str << "\nMetadata: " << downcastMetadata(s.metadata_).toString();
  return str;
}

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = cos(yaw / 2);
  q.z = sin(yaw / 2);
  q.x = 0;
  q.y = 0;
  return q;
}

inline geometry_msgs::msg::Pose makePose(const double x, const double y, const double theta)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation = createQuaternionMsgFromYaw(theta);
  return p;
}

using std::string;

bool contains(const string& s1, const string& s2)
{
  return strstr(s1.c_str(), s2.c_str()) != NULL;
}
