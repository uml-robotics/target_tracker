/*
 * Copyright (c) 2013, University of Massachusetts Lowell.
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

/* Author: Mikhail Medvedev */

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <XmlRpcException.h>

#include "target.h"
#include "target_visualizer.h"

namespace target_tracker
{
/**
 * Given a list of targets on the plane, track which ones have been reached.
 * Publish the PoseArray message containing the targets that are still
 * active. The targets are transformed into base_frame_id frame.
 */

class TargetTracker
{

public:
  TargetTracker();
  void loadTargets(std::string targets_parameter_name = "targets");
  void update();

protected:
  Target::vector targets_;
  TargetVisualizer vis_;
  tf::TransformListener tf_listener_;
  std::string map_frame_id_;
  std::string base_frame_id_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_targets_;

  void targetsCb(const geometry_msgs::PoseArrayConstPtr & msg);
};

}
/// namespace target_tracker

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "target_tracker");
  target_tracker::TargetTracker _tt;
  ros::Rate r(10);
  while (ros::ok() && ! ros::isShuttingDown())
  {
    _tt.update();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

namespace target_tracker
{

TargetTracker::TargetTracker() :
    nh_("~"),
    pub_(nh_.advertise<geometry_msgs::PoseArray>("/target_poses", 5)),
    sub_targets_(nh_.subscribe("init_targets",3 , &TargetTracker::targetsCb, this))
{
  ros::NodeHandle private_nh("~");
  private_nh.param("base_frame_id", base_frame_id_, (std::string)"/base_link");
  private_nh.param("map_frame_id", map_frame_id_, (std::string)"/map");

  while (
      ros::ok() && ! ros::isShuttingDown() &&
      !tf_listener_.waitForTransform(map_frame_id_, base_frame_id_,
                                        ros::Time::now(), ros::Duration(1.0)))
  {
    ROS_INFO_STREAM(
        "Waiting for transform "<<map_frame_id_<<"->"<<base_frame_id_<<".");
  }
  ROS_INFO("Transform ok.");

  loadTargets();
}

/**
 * Load targets from parameter server.
 * @param targets_parameter_name
 */
void TargetTracker::loadTargets(std::string targets_parameter_name)
{
  ros::NodeHandle private_nh("~");
  if (private_nh.hasParam(targets_parameter_name))
  {
    try
    {
      ROS_INFO_STREAM(
          "Loading targets from '"<< targets_parameter_name <<"' parameter.");
      XmlRpc::XmlRpcValue value;
      private_nh.getParam(targets_parameter_name, value);
      ROS_ASSERT(value.getType() == XmlRpc::XmlRpcValue::TypeArray);

      for (int i = 0; i < value.size(); i++)
      {
        ROS_ASSERT(value[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
        if (value[i].size() == 3)
        {
          XmlRpc::XmlRpcValue v = value[i];
          ROS_ASSERT(v[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          ROS_ASSERT(v[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          ROS_ASSERT(v[2].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          targets_.push_back(Target(v[0], v[1], v[2]));
        }
        else
        {
          ROS_WARN_STREAM(
              "Skipping incomplete target on line " << i <<", only found " << value[i].size()<<" elements (3 expected)");
        }
      }
    }
    catch (XmlRpc::XmlRpcException & ex)
    {
      ROS_ERROR_STREAM("Error parsing parameter: "<<ex.getMessage());
    }
  }
  else
  {
    ROS_WARN_STREAM(
        "Parameter '"<< targets_parameter_name <<"' not found, no targets were loaded.");
  }
  vis_.setTargets(&targets_);
}

inline double magnitude(geometry_msgs::Point point)
{
  return sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
}

void TargetTracker::update()
{
  geometry_msgs::PoseStamped base_pose;
  base_pose.header.stamp = ros::Time::now();

  geometry_msgs::PoseArray targets_in_base_frame;
  targets_in_base_frame.header.stamp = base_pose.header.stamp;
  targets_in_base_frame.header.frame_id = base_frame_id_;

  geometry_msgs::PoseStamped map_pose;
  map_pose.header.frame_id = map_frame_id_;
  try
  {
    tf_listener_.waitForTransform(map_frame_id_, base_frame_id_,
                                  ros::Time::now(), ros::Duration(0.5));

    for (auto target = targets_.begin(); target != targets_.end(); ++target)
    { // Transform target poses into base frame
      if (target->isActive())
      {
        map_pose.pose = target->getPose();
        tf_listener_.transformPose(base_frame_id_, map_pose, base_pose);
        targets_in_base_frame.poses.push_back(base_pose.pose);
        if (magnitude(base_pose.pose.position) < target->radius_)
        {
          target->setActive(false);
        }
      }
    }
    pub_.publish(targets_in_base_frame);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  vis_.publish();
}

inline void TargetTracker::targetsCb(
    const geometry_msgs::PoseArrayConstPtr& msg)
{
  targets_.clear();
  for (auto pose = msg->poses.begin(); pose != msg->poses.end(); ++pose)
  {
    targets_.push_back(
        Target(pose->position.x, pose->position.y, pose->position.z));
  }
}

}
