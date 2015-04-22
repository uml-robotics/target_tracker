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
/* Rendered useless for those without non-public giblets by Eric McCann. wuuups! */

#include <vector>
#include <boost/thread/mutex.hpp>
#include "XmlRpcValue.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <rlucid_msgs/TargetArray.h>
#include <std_msgs/Byte.h>
#include <tf/transform_listener.h>
#include <XmlRpcException.h>

#include "target.h"
#include "target_visualizer.h"
#include "visibility_checker.h"

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#define toMap(vector_type_argument, type_argument, list_of_dynparams, map_name) \
  for (std::vector< vector_type_argument >::const_iterator bp = list_of_dynparams.begin(); bp != list_of_dynparams.end(); bp++) \
  { \
    map_name[bp->name] = (type_argument)bp->value; \
  } \
  while(0)

using namespace XmlRpc;

namespace target_tracker
{
/**
 * Given a list of targets on the plane, track which ones have been reached.
 * Publish the PoseArray message containing the targets that are still
 * active. The targets are transformed into base_frame_id frame.
 *
 * It also checks the /map used for navigation and determines if the target should
 * be visible on that map. Only clears visible targets.
 */

class TargetTracker
{

public:
  TargetTracker();
  void loadTargets();
  void update();

  /**
   * Publish number of remaining active targets
   */
  void publishActiveCount();

protected:
  bool in_progress_;
  bool paused_;
  std::string current_map_;
  std::map<int,std::string> map_names;
  std::map<std::string, TargetStorage::vector> targets_;
  TargetVisualizer vis_;
  tf::TransformListener tf_listener_;
  std::string map_frame_id_;
  std::string base_frame_id_;
  ros::NodeHandle nh_;
  ros::Subscriber experimentSub;
  ros::Publisher pub_targets_;
  ros::Publisher pub_poses_;
  ros::Publisher pub_count_;
  ros::Publisher pub_nearest_target_;
  
  VisibilityChecker visibility_checker_;
  bool use_visibility_check_;

  // Support for manual target clearing.
  bool manual_clearing_;
  std::string manual_clearing_topic_;
  ros::Subscriber sub_manual_clear_;
  std::vector<geometry_msgs::Pose> pose_to_clear_;
  boost::mutex pose_to_clear_mutex_;

  void manualClearCb(const geometry_msgs::PoseConstPtr & msg);
  void experimentCallback(const dynamic_reconfigure::Config &cfg);
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
    tf_listener_(),
    in_progress_(false),
    paused_(false),
    map_names(),
    current_map_(""),
    experimentSub(nh_.subscribe("/experiment/parameter_updates", 1, &TargetTracker::experimentCallback, this)),
    pub_targets_(nh_.advertise<rlucid_msgs::TargetArray>("/target_poses", 5)),
    pub_poses_(nh_.advertise<geometry_msgs::PoseArray>("/target_posearray", 5)),
    pub_count_(nh_.advertise<std_msgs::Byte>("/active_targets_count", 5, true)),
    pub_nearest_target_(nh_.advertise<rlucid_msgs::Target>("/nearest_target", 100, true))
{
  nh_.param("base_frame_id", base_frame_id_, (std::string)"/base_link");
  nh_.param("map_frame_id", map_frame_id_, (std::string)"/map");
  nh_.param("use_visibility_check", use_visibility_check_, true);
  nh_.param("manual_clearing", manual_clearing_, true);
  nh_.param("manual_clearing_topic", manual_clearing_topic_, (std::string)"/target_to_remove");

  // DANGER WILL ROBINSON
  // THIS CODE IS DESIGNED FOR A VERY SPECIFIC YAML FORMAT
  // IT WILL PROBABLY EAT YOUR CAT IF YOU USE IT
  // OR CURSE YOU WITH SOME SORT OF MUMMY CURSE (... or something) IF YOU TRY TO READ IT
  //    IT RELIES ON A PRIVATE REPOSITORY'S DYNAMIC RECONFIGURE SERVER TO CONTROL WHICH MAP'S TARGETS IT IS TRACKING
/*
//example of yaml format
{
  configs: {
    a: { //map identifier
      resolution: 0.0127, // m/pix
      width: 842, // map image width pix
      height: 728, // map image height pix
      north_angle: 1.57, //radians, relative to image
      targets: {  // target_identifier: [ x pixels, y pixels, radians relative to image]
        a: [321.0, 120.0, 3.14],
        b: [522.0, 120.0, 3.14],
        c: [723.0, 264.0, 1.57],
        d: [723.0, 465.0, 0.00],
        e: [522.0, 609.0, 3.14],
        f: [321.0, 609.0, 0.00],
        g: [120.0, 465.0, 1.57],
        h: [120.0, 264.0, 3.14]
      }
    },
    {
      // and so on and so on until all you base are belong to us
    }
  }
}
*/
  XmlRpcValue val;
  if (nh_.hasParam("configs") && nh_.getParam("configs", val) && val.getType() == XmlRpcValue::TypeStruct)
  {
    XmlRpcValue map_kvp;
    int map_index=0;
    for (XmlRpcValue::iterator map_kvp = val.begin();
          map_kvp != val.end();
          map_kvp++)
    {
      map_names[map_index++]=map_kvp->first;
      double north_angle = map_kvp->second["north_angle"];
      double resolution = map_kvp->second["resolution"];
      double height = (double)((int)map_kvp->second["height"]);
      for (XmlRpcValue::iterator room_kvp = map_kvp->second["targets"].begin();
            room_kvp != map_kvp->second["targets"].end();
            room_kvp++)
      {
        ROS_INFO("Loading target for M(%s)R(%s)", map_kvp->first.c_str(), room_kvp->first.c_str());
        double x,y,t;
        x = (double)room_kvp->second[0]*resolution;
        y = (height - (double)room_kvp->second[1])*resolution;
        t = (double)(room_kvp->second[2]);
        tf::Quaternion quat;
        geometry_msgs::Quaternion q;
        quat.setRPY(0, 0, t);
        tf::quaternionTFToMsg(quat, q);
        TargetStorage ts(x, y, 1.0, q);
        targets_[map_kvp->first].push_back(ts);
      }
    }
  }
  else
  {
    ROS_WARN_STREAM(
        "Parameter configs not found, no targets were loaded.");
  }

  if (manual_clearing_)
  {
    sub_manual_clear_ = nh_.subscribe(manual_clearing_topic_, 5, &TargetTracker::manualClearCb, this);
  }
  while (
      ros::ok() && ! ros::isShuttingDown() &&
      !tf_listener_.waitForTransform(map_frame_id_, base_frame_id_,
                                        ros::Time(0), ros::Duration(1.0)))
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
void TargetTracker::loadTargets()
{
  ROS_INFO("Loading targets for MAP=\"%s\"", current_map_.c_str());
  if (current_map_.size()==0)
    return;
  vis_.setTargets(&(targets_[current_map_]));
  update();
}

inline double magnitude(geometry_msgs::Point point)
{
  return sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
}

void TargetTracker::update()
{
  if (current_map_.size()==0 || !in_progress_ || paused_)
    return;
  rlucid_msgs::Target new_target;
  geometry_msgs::PoseStamped base_pose;
  rlucid_msgs::Target nearest;
  double dist=0, nearestdistance=1000.0;  //if the targets are more than 1 KM away, then you're SOL
  base_pose.header.stamp = ros::Time::now();

  rlucid_msgs::TargetArray targets_in_base_frame;
  geometry_msgs::PoseArray poses_in_base_frame;
  poses_in_base_frame.header.stamp = targets_in_base_frame.header.stamp = base_pose.header.stamp;
  poses_in_base_frame.header.frame_id = targets_in_base_frame.header.frame_id = base_frame_id_;

  geometry_msgs::PoseStamped map_pose;
  map_pose.header.frame_id = map_frame_id_;
  try
  {
    if (!tf_listener_.waitForTransform(map_frame_id_, base_frame_id_,
                                  ros::Time(0), ros::Duration(0.5))) {
      ROS_WARN("failed to look up transform from %s to %s... prognosis is grim.", map_frame_id_.c_str(), base_frame_id_.c_str());
    }
  
    bool new_pose = false;
    geometry_msgs::Pose to_clear;
    {
      boost::mutex::scoped_lock lock(pose_to_clear_mutex_);
      if (not pose_to_clear_.empty())
      {
        to_clear = *(pose_to_clear_.end() - 1);
        ROS_INFO_STREAM("ACTUALLY CLEARING " << to_clear); 
        pose_to_clear_.clear();
        new_pose = true;
      }
    }
    for (auto target = targets_[current_map_].begin(); target != targets_[current_map_].end(); ++target)
    { // Transform target poses into base frame
      if (true)
      {
        map_pose.pose = target->getPose();
        tf_listener_.transformPose(base_frame_id_, map_pose, base_pose);
        new_target.pose = base_pose.pose;
        new_target.cleared_count = target->getClearedCount();
        targets_in_base_frame.targets.push_back(new_target);
        if ((dist=magnitude(new_target.pose.position))<nearestdistance)
        {
          nearestdistance=dist;
          nearest.pose = new_target.pose;
          nearest.cleared_count = new_target.cleared_count;
        }
        poses_in_base_frame.poses.push_back(base_pose.pose);
        if (not manual_clearing_)
        {
          if (magnitude(base_pose.pose.position) < target->radius_
              and (not use_visibility_check_ or visibility_checker_.targetIsVisible(*target)))
          {
            target->incrementClearedCount();
            publishActiveCount();
          }
        }

        else // manual_clearing
        {
          geometry_msgs::Point distance_vector;
          
          if (new_pose)
          {
            distance_vector.x = to_clear.position.x - base_pose.pose.position.x;
            distance_vector.y = to_clear.position.y - base_pose.pose.position.y;
            distance_vector.z = to_clear.position.z - base_pose.pose.position.z;
            if (magnitude(distance_vector) < 2.0)
            {
              target->incrementClearedCount();
              publishActiveCount();
            }
            else
            {
              ROS_WARN_STREAM("CLEARED TARGET IS " << magnitude(distance_vector) << " AWAY");
            }
          }
        }
      }
    }
    if (nearestdistance < 1000.0) {
      pub_nearest_target_.publish(nearest);
    }
    pub_targets_.publish(targets_in_base_frame);
    pub_poses_.publish(poses_in_base_frame);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  vis_.publish();
}

void TargetTracker::publishActiveCount()
{
  if (current_map_.size()==0 || !in_progress_ || paused_)
    return;
  std_msgs::Byte count;
  for (auto target = targets_[current_map_].begin(); target != targets_[current_map_].end(); ++target)
    {
      if (target->getClearedCount() == 0)
      {
        count.data++;
      }
    }
  pub_count_.publish(count);
}



void TargetTracker::experimentCallback(const dynamic_reconfigure::Config &config)
{
  std::map<std::string, bool> bools;
  std::map<std::string, int> ints;
  toMap(dynamic_reconfigure::BoolParameter, bool, config.bools, bools);
  toMap(dynamic_reconfigure::IntParameter, int, config.ints, ints);
  paused_ = bools["paused"];
  in_progress_ = bools["in_progress"];
  int mapindex = ints["preset_id"];
  std::string current_map;
  if (map_names.find(mapindex) != map_names.end())
    current_map = map_names[mapindex];
  else
    current_map = "";
  if (current_map.compare(current_map_) != 0) {
    ROS_INFO("MAP CHANGED: map=%s, paused=%s, in_progress=%s", current_map.size() == 0 ? "?" : current_map.c_str(), paused_ ? "T" : "F", in_progress_ ? "T" : "F");
    current_map_ = current_map;
    loadTargets();
  }
  if (!in_progress_)
  {
    ROS_WARN("NOT IN PROGRESS ==> RESETTING TARGETS COUNT");
    for(auto amap = targets_.begin(); amap != targets_.end(); amap++)
    {
      for(auto atarget = amap->second.begin(); atarget != amap->second.end(); atarget++)
      {
        atarget->setClearedCount(0);
      }
    }
  }
}

inline void TargetTracker::manualClearCb(const geometry_msgs::PoseConstPtr& msg)
{
  ROS_INFO_STREAM("Incrementing clear count on " << *msg.get());
  boost::mutex::scoped_lock lock(pose_to_clear_mutex_);
  pose_to_clear_.push_back(*msg);
}

}
