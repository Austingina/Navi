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

#ifndef DWB_CRITICS__HEADING_ALIGN_HPP_
#define DWB_CRITICS__HEADING_ALIGN_HPP_

#include "dwb_core/trajectory_critic.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"

namespace dwb_critics
{
/**
 * @class HeadingAlignCritic
 * @brief Penalize trajectories where robot heading differs from path direction
 *
 * This critic calculates the angular difference between the robot's heading at the end
 * of a trajectory and the direction of the global path at the closest point.
 * It encourages the robot to face along the path direction, which is especially useful
 * for omnidirectional robots that cannot move backward.
 */
class HeadingAlignCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

protected:
  /**
   * @brief Calculate path direction at a given point by looking at nearby path points
   * @param pose The pose to find path direction for
   * @return The path direction in radians
   */
  double getPathDirection(const geometry_msgs::msg::Pose2D & pose);

  /**
   * @brief Normalize angle to [-pi, pi]
   */
  double normalizeAngle(double angle);

  nav_2d_msgs::msg::Path2D global_plan_;
  double forward_point_distance_;  ///< How far ahead to look for path direction
  double goal_distance_threshold_;  ///< Disable critic when close to goal
  bool zero_scale_;
};
}  // namespace dwb_critics

#endif  // DWB_CRITICS__HEADING_ALIGN_HPP_
