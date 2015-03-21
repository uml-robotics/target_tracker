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

#ifndef VISIBILITY_CHECKER_H_
#define VISIBILITY_CHECKER_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

#include "target.h"
namespace target_tracker
{
/**
 * For a target, check if it is visible in a context of the map used for localization.
 */
class VisibilityChecker
{
private:
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  std::string robot_frame_id_; // Frame id to be used to calculate robot's location.
  ros::Subscriber sub_map_;
  nav_msgs::OccupancyGrid map_;

public:
  VisibilityChecker();

  /**
   *
   * @param target Target to check, assumed map frame.
   * @return True if target should be visible from robot position in the localization map.
   */
  bool targetIsVisible(TargetStorage & target);

private:
  void mapCb(const nav_msgs::OccupancyGridConstPtr& msg);
};

} /* namespace target_tracker */

#endif /* VISIBILITY_CHECKER_H_ */
