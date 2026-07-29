/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "heading_align_critic/heading_align.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "angles/angles.h"

namespace heading_align_critic
{

void HeadingAlignCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Get parameters
  forward_point_distance_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".forward_point_distance", 0.5);

  xy_goal_tolerance_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".xy_goal_tolerance", 0.25);
}

bool HeadingAlignCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & /*vel*/,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  global_plan_ = global_plan;

  // Disable when close to goal to avoid instability
  double dx = pose.x - goal.x;
  double dy = pose.y - goal.y;
  double sq_dist = dx * dx + dy * dy;
  if (sq_dist < xy_goal_tolerance_ * xy_goal_tolerance_) {
    zero_scale_ = true;
  } else {
    zero_scale_ = false;
  }

  return true;
}

double HeadingAlignCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  if (zero_scale_ || global_plan_.poses.empty() || traj.poses.empty()) {
    return 0.0;
  }

  // Get the last pose of the trajectory
  const auto & end_pose = traj.poses.back();

  // Get the path direction at the trajectory end point
  double path_direction = getPathDirection(end_pose);

  // Calculate angular difference between robot heading and path direction
  double heading_diff = angles::shortest_angular_distance(end_pose.theta, path_direction);

  // Base score: absolute angular difference (0 to pi)
  // This penalizes trajectories where robot doesn't face along the path
  double score = fabs(heading_diff);

  // When turning (angular velocity > 0.1), penalize high xy velocity
  // to encourage slower linear speed during turns
  double angular_vel = fabs(traj.velocity.theta);
  if (angular_vel > 0.1) {
    double xy_vel = std::sqrt(traj.velocity.x * traj.velocity.x + traj.velocity.y * traj.velocity.y);
    // Subtract penalty: higher xy_vel and angular_vel means lower score (less preferred)
    // This effectively increases cost for high-speed turns
    score += xy_vel * angular_vel;
  }

  return score;
}

double HeadingAlignCritic::getPathDirection(const geometry_msgs::msg::Pose2D & pose)
{
  if (global_plan_.poses.size() < 2) {
    return pose.theta;  // Not enough points, use current heading
  }

  // Find the closest point on the path
  double min_dist = std::numeric_limits<double>::max();
  size_t closest_idx = 0;

  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    double dx = pose.x - global_plan_.poses[i].x;
    double dy = pose.y - global_plan_.poses[i].y;
    double dist = dx * dx + dy * dy;
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  // Find a point ahead on the path to calculate direction
  size_t ahead_idx = closest_idx;
  double accumulated_dist = 0.0;

  for (size_t i = closest_idx; i < global_plan_.poses.size() - 1; ++i) {
    double dx = global_plan_.poses[i + 1].x - global_plan_.poses[i].x;
    double dy = global_plan_.poses[i + 1].y - global_plan_.poses[i].y;
    accumulated_dist += std::sqrt(dx * dx + dy * dy);
    ahead_idx = i + 1;
    if (accumulated_dist >= forward_point_distance_) {
      break;
    }
  }

  // Calculate path direction from closest point to ahead point
  double dx = global_plan_.poses[ahead_idx].x - global_plan_.poses[closest_idx].x;
  double dy = global_plan_.poses[ahead_idx].y - global_plan_.poses[closest_idx].y;

  if (dx == 0.0 && dy == 0.0) {
    // Points are the same, use the path pose's theta if available
    return global_plan_.poses[closest_idx].theta;
  }

  return std::atan2(dy, dx);
}

double HeadingAlignCritic::normalizeAngle(double angle)
{
  return angles::normalize_angle(angle);
}

}  // namespace heading_align_critic

PLUGINLIB_EXPORT_CLASS(heading_align_critic::HeadingAlignCritic, dwb_core::TrajectoryCritic)
