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
#include "target_visualizer.h"

#include <visualization_msgs/MarkerArray.h> // TargetVisualizer

namespace target_tracker
{

TargetVisualizer::TargetVisualizer() :
        pub_(
            nh_.advertise<visualization_msgs::MarkerArray>(
                "/visualization_marker_array", 5))
{
}

void TargetVisualizer::publish()
{
  visualization_msgs::MarkerArray arr;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.ns = "targets";
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.scale.z = 0.01;
  marker.color.a = 0.5;
  for (auto target = targets_->begin(); target != targets_->end(); ++target)
  {
    marker.scale.x = target->radius_ * 2;
    marker.scale.y = target->radius_ * 2;
    marker.pose = target->getPose();
    marker.color.r = target->isActive() ? 0.5 : 0;
    marker.color.b = target->isActive() ? 0 : 0.5;
    arr.markers.push_back(marker);
    marker.id++;
  }
  ROS_DEBUG_STREAM("Publishing " << arr.markers.size() << " markers");
  pub_.publish(arr);
}

void TargetVisualizer::setTargets(Target::vector * targets)
{
  targets_ = targets;
}
} /* namespace target_tracker */
