/*
 * Copyright (c) 2018, the neonavigation authors
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
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#ifndef TRAJECTORY_TRACKER_MSGS__CONVERTER_HPP_
#define TRAJECTORY_TRACKER_MSGS__CONVERTER_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <trajectory_tracker_msgs/msg/path_with_velocity.hpp>

namespace trajectory_tracker_msgs
{
inline msg::PathWithVelocity toPathWithVelocity(const nav_msgs::msg::Path & src, const double vel)
{
  msg::PathWithVelocity dest;
  dest.header = src.header;
  dest.poses.clear();
  dest.poses.reserve(src.poses.size());
  for (const geometry_msgs::msg::PoseStamped & p : src.poses) {
    msg::PoseStampedWithVelocity pv;
    pv.header = p.header;
    pv.pose = p.pose;
    pv.linear_velocity.x = vel;
    dest.poses.push_back(pv);
  }
  return dest;
}
inline msg::PathWithVelocity toPathWithVelocity(
  const nav_msgs::msg::Path & src,
  const geometry_msgs::msg::Vector3 & vel)
{
  msg::PathWithVelocity dest;
  dest.header = src.header;
  dest.poses.clear();
  dest.poses.reserve(src.poses.size());
  for (const geometry_msgs::msg::PoseStamped & p : src.poses) {
    msg::PoseStampedWithVelocity pv;
    pv.header = p.header;
    pv.pose = p.pose;
    pv.linear_velocity = vel;
    dest.poses.push_back(pv);
  }
  return dest;
}
}  // namespace trajectory_tracker_msgs

#endif  // TRAJECTORY_TRACKER_MSGS__CONVERTER_HPP_
