/*
 * Copyright (c) 2014, University of Massachusetts Lowell.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of University of Massachusetts Lowell. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Mikhail Medvedev <mmedvede@cs.uml.edu> */

#include "visibility_checker.h"
namespace raytrace
{

/**
 * Return true if there is wall at the pixel location or out of bounds
 */
bool pixelIsWall(nav_msgs::OccupancyGrid & map, int y, int x)
{
  int index = map.info.width * y + x;
  return (index < 0 or index >= map.data.size() or map.data[index] == 100);
}

/**
 * Determine if the line crosses the wall on the map.
 * @details Bitmap/Bresenham's line algorithm
 * source: http://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C.2B.2B
 * with modifications to fit our needs.
 */
bool isObscured(nav_msgs::OccupancyGrid & map, float x1, float y1, float x2, float y2)
{
  // Bresenham's line algorithm
  const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
  if (steep)
  {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }

  bool swapped = false;
  if (x1 > x2)
  {
    std::swap(x1, x2);
    std::swap(y1, y2);
    swapped = true;
  }

  const float dx = x2 - x1;
  const float dy = fabs(y2 - y1);

  float error = dx / 2.0f;
  const int ystep = (y1 < y2) ? 1 : -1;
  int y = (int)y1;

  const int maxX = (int)x2;

  bool is_wall = false;
  for (int x = (int)x1; x < maxX and not is_wall; x++)
  {
    if (steep)
    {
      is_wall = pixelIsWall(map, y, x);
    }
    else
    {
      is_wall = pixelIsWall(map, x, y);
    }

    error -= dy;
    if (error < 0)
    {
      y += ystep;
      error += dx;
    }
  }
  return is_wall;
}
}

namespace target_tracker
{

double getIndex(double x0, double resolution, double x)
{
  return (x - x0) / resolution;
}

geometry_msgs::PoseStamped getPose(const std::string ref_frame_id, const std::string frame_id,
                                   const tf::TransformListener & tf)
{
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now() - ros::Duration(0.1); // Add a little tolerance (because of "extraploation in to the future" issue)
  pose.header.frame_id = frame_id;
  pose.pose.orientation.w = 1;
  tf::StampedTransform tr;
  try
  {
    if (!tf.waitForTransform(ref_frame_id, pose.header.frame_id, pose.header.stamp, ros::Duration(0.1)))
      ROS_WARN("Could not transform in time");
    tf.transformPose(ref_frame_id, pose, pose);
  }
  catch (tf::TransformException & e)
  {
    ROS_WARN_STREAM(e.what());
  }
  return pose;
}

VisibilityChecker::VisibilityChecker() :
    sub_map_(nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &VisibilityChecker::mapCb, this)) //
{
  ros::NodeHandle private_nh("~");
  private_nh.param("robot_frame_id", robot_frame_id_, (std::string)"/base_link");
}

bool VisibilityChecker::targetIsVisible(Target& target)
{
  if (map_.data.size() == 0)
  {
    // If no map is available, assume target is visible
    return true;
  }

  geometry_msgs::PoseStamped robot_pose = getPose( //
      "/map", robot_frame_id_, tf_listener_);

  float & res = map_.info.resolution;
  geometry_msgs::Point & origin = map_.info.origin.position;

  // Robot
  geometry_msgs::Point & pos = robot_pose.pose.position;
  int rx = getIndex(origin.x, res, pos.x);
  int ry = getIndex(origin.y, res, pos.y);

  // Target
  geometry_msgs::Pose target_pose = target.getPose();
  int tx = getIndex(origin.x, res, target_pose.position.x);
  int ty = getIndex(origin.y, res, target_pose.position.y);

  return not raytrace::isObscured(map_, ry, rx, ty, tx);
}

void VisibilityChecker::mapCb(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map_ = *msg;
  ROS_INFO_STREAM( //
      "Map received, width:" << map_.info.width//
      << " height:" << map_.info.height//
      << " resolution:" << map_.info.resolution);
}
} /* namespace target_tracker */
