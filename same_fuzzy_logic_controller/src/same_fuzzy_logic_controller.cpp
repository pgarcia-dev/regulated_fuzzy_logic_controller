// Copyright (c) 2023 Pablo García. pgarcia.developer@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the “License”);
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an “AS IS” BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <cmath>

#include "same_fuzzy_logic_controller/same_fuzzy_logic_controller.hpp"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "dwb_msgs/msg/critic_score.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <tf2/transform_datatypes.h>

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace same_fuzzy_logic_controller
{

SameFuzzyLogicController::SameFuzzyLogicController()
: traj_gen_loader_("dwb_core", "dwb_core::TrajectoryGenerator"),
  critic_loader_("dwb_core", "dwb_core::TrajectoryCritic")
{
}

void SameFuzzyLogicController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();

  logger_ = node->get_logger();
  clock_ = node->get_clock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  dwb_plugin_name_ = name;
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".critics",
    rclcpp::PARAMETER_STRING_ARRAY);
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".default_critic_namespaces",
    rclcpp::ParameterValue(std::vector<std::string>()));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".prune_plan",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".prune_distance",
    rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".debug_trajectory_details",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".trajectory_generator_name",
    rclcpp::ParameterValue(std::string("dwb_plugins::StandardTrajectoryGenerator")));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".transform_tolerance",
    rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".shorten_transformed_plan",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".short_circuit_trajectory_evaluation",
    rclcpp::ParameterValue(true));

  std::string traj_generator_name;

  double transform_tolerance;
  node->get_parameter(dwb_plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
  RCLCPP_INFO(logger_, "Setting transform_tolerance to %f", transform_tolerance);

  node->get_parameter(dwb_plugin_name_ + ".prune_plan", prune_plan_);
  node->get_parameter(dwb_plugin_name_ + ".prune_distance", prune_distance_);
  node->get_parameter(dwb_plugin_name_ + ".debug_trajectory_details", debug_trajectory_details_);
  node->get_parameter(dwb_plugin_name_ + ".trajectory_generator_name", traj_generator_name);
  node->get_parameter(
    dwb_plugin_name_ + ".short_circuit_trajectory_evaluation",
    short_circuit_trajectory_evaluation_);
  node->get_parameter(dwb_plugin_name_ + ".shorten_transformed_plan", shorten_transformed_plan_);

  pub_ = std::make_unique<dwb_core::DWBPublisher>(node, dwb_plugin_name_);
  pub_->on_configure();

  traj_generator_ = traj_gen_loader_.createUniqueInstance(traj_generator_name);

  traj_generator_->initialize(node, dwb_plugin_name_);

  try {
    loadCritics();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Couldn't load critics! Caught exception: %s", e.what());
    throw nav2_core::ControllerException(
            "Couldn't load critics! Caught exception: " +
            std::string(e.what()));
  }
}

void
SameFuzzyLogicController::activate()
{
  pub_->on_activate();
}

void
SameFuzzyLogicController::deactivate()
{
  pub_->on_deactivate();
}

void
SameFuzzyLogicController::cleanup()
{
  pub_->on_cleanup();

  traj_generator_.reset();
}

std::string
SameFuzzyLogicController::resolveCriticClassName(std::string base_name)
{
  if (base_name.find("Critic") == std::string::npos) {
    base_name = base_name + "Critic";
  }

  if (base_name.find("::") == std::string::npos) {
    for (unsigned int j = 0; j < default_critic_namespaces_.size(); j++) {
      std::string full_name = default_critic_namespaces_[j] + "::" + base_name;
      if (critic_loader_.isClassAvailable(full_name)) {
        return full_name;
      }
    }
  }
  return base_name;
}

void
SameFuzzyLogicController::loadCritics()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(dwb_plugin_name_ + ".default_critic_namespaces", default_critic_namespaces_);
  if (default_critic_namespaces_.empty()) {
    default_critic_namespaces_.emplace_back("dwb_critics");
  }

  std::vector<std::string> critic_names;
  if (!node->get_parameter(dwb_plugin_name_ + ".critics", critic_names)) {
    throw std::runtime_error("No critics defined for " + dwb_plugin_name_);
  }

  node->get_parameter(dwb_plugin_name_ + ".critics", critic_names);
  for (unsigned int i = 0; i < critic_names.size(); i++) {
    std::string critic_plugin_name = critic_names[i];
    std::string plugin_class;

    declare_parameter_if_not_declared(
      node, dwb_plugin_name_ + "." + critic_plugin_name + ".class",
      rclcpp::ParameterValue(critic_plugin_name));
    node->get_parameter(dwb_plugin_name_ + "." + critic_plugin_name + ".class", plugin_class);

    plugin_class = resolveCriticClassName(plugin_class);

    dwb_core::TrajectoryCritic::Ptr plugin = critic_loader_.createUniqueInstance(plugin_class);
    RCLCPP_INFO(
      logger_,
      "Using critic \"%s\" (%s)", critic_plugin_name.c_str(), plugin_class.c_str());
    critics_.push_back(plugin);
    try {
      plugin->initialize(node, critic_plugin_name, dwb_plugin_name_, costmap_ros_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Couldn't initialize critic plugin!");
      throw nav2_core::ControllerException(
              "Couldn't initialize critic plugin: " +
              std::string(e.what()));
    }
    RCLCPP_INFO(logger_, "Critic plugin initialized");
  }
}

void
SameFuzzyLogicController::setPlan(const nav_msgs::msg::Path & path) ////////////////////////////////////////////////////////////////////////////////////////
{
// RCLCPP_INFO(logger_, "next paths's position - x: %f , y: %f, z: %f, vector size: %ld", path.poses.at(0).pose.position.x, path.poses.at(0).pose.position.y, path.poses.at(0).pose.position.z, path.poses.size());
 // RCLCPP_INFO(logger_, "next paths's orientation - x: %f , y: %f, z: %f, w: %f", path.poses.at(0).pose.orientation.x, path.poses.at(0).pose.orientation.y, path.poses.at(0).pose.orientation.z, path.poses.at(0).pose.orientation.w);

//  next_waypoint_x_ = path.poses.back().pose.position.x;
//  next_waypoint_y_ = path.poses.back().pose.position.y;
  next_waypoint_x_ = path.poses.at(0).pose.position.x;
  next_waypoint_y_ = path.poses.at(0).pose.position.y;

  auto path2d = nav_2d_utils::pathToPath2D(path);
  for (dwb_core::TrajectoryCritic::Ptr & critic : critics_) {
    critic->reset();
  }

  traj_generator_->reset();

  pub_->publishGlobalPlan(path2d);
  global_plan_ = path2d;
}

geometry_msgs::msg::TwistStamped
SameFuzzyLogicController::computeVelocityCommands( ///////////////////////////////////////////////////////////////////////////////////////////////
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  //RCLCPP_INFO(logger_, "*** robot's position - x: %f , y: %f, z: %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  //RCLCPP_INFO(logger_, "robot's orientation - x: %f , y: %f, z: %f, w: %f", pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

  tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  //RCLCPP_INFO(logger_, "*** robot's orientation - roll: %f , pitch: %f, yaw: %f", roll, pitch, yaw);

  double dx = next_waypoint_x_ - pose.pose.position.x; 
  double dy = next_waypoint_y_ - pose.pose.position.y; 

  double angle = atan2(dy, dx);

  double heading = angle - yaw;     
  //and here I just make sure my angle is between minus pi and pi! 
  if (heading > M_PI)
    heading -= (2*M_PI);
  if (heading <= -M_PI)
    heading += 2*M_PI;

  
  if (heading > 0) //////////////// temporal until using TF2
    heading -= M_PI;
  else
    heading += M_PI;

  RCLCPP_INFO(logger_, "*** %f",heading);
  //RCLCPP_INFO(logger_, "***1 robot's position - x: %f , y: %f, z: %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  //RCLCPP_INFO(logger_, "***2 next paths's position - x: %f , y: %f", next_waypoint_x_, next_waypoint_y_);




  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results = nullptr;
  if (pub_->shouldRecordEvaluation()) {
    results = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
  }

  try {
    nav_2d_msgs::msg::Twist2DStamped cmd_vel2d = computeVelocityCommands(
      nav_2d_utils::poseStampedToPose2D(pose),
      nav_2d_utils::twist3Dto2D(velocity), results);
    pub_->publishEvaluation(results);
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.twist = nav_2d_utils::twist2Dto3D(cmd_vel2d.velocity);
    return cmd_vel;
  } catch (const nav2_core::ControllerTFError & e) {
    pub_->publishEvaluation(results);
    throw e;
  } catch (const nav2_core::InvalidPath & e) {
    pub_->publishEvaluation(results);
    throw e;
  } catch (const nav2_core::NoValidControl & e) {
    pub_->publishEvaluation(results);
    throw e;
  } catch (const nav2_core::ControllerException & e) {
    pub_->publishEvaluation(results);
    throw e;
  }
}

void
SameFuzzyLogicController::prepareGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose, nav_2d_msgs::msg::Path2D & transformed_plan,
  nav_2d_msgs::msg::Pose2DStamped & goal_pose, bool publish_plan)
{
  transformed_plan = transformGlobalPlan(pose);
  if (publish_plan) {
    pub_->publishTransformedPlan(transformed_plan);
  }

  goal_pose.header.frame_id = global_plan_.header.frame_id;
  goal_pose.pose = global_plan_.poses.back();
  nav_2d_utils::transformPose(
    tf_, costmap_ros_->getGlobalFrameID(), goal_pose,
    goal_pose, transform_tolerance_);
}

nav_2d_msgs::msg::Twist2DStamped
SameFuzzyLogicController::computeVelocityCommands(
  const nav_2d_msgs::msg::Pose2DStamped & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  if (results) {
    results->header.frame_id = pose.header.frame_id;
    results->header.stamp = clock_->now();
  }

  nav_2d_msgs::msg::Path2D transformed_plan;
  nav_2d_msgs::msg::Pose2DStamped goal_pose;

  prepareGlobalPlan(pose, transformed_plan, goal_pose);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  for (dwb_core::TrajectoryCritic::Ptr & critic : critics_) {
    if (!critic->prepare(pose.pose, velocity, goal_pose.pose, transformed_plan)) {
      RCLCPP_WARN(rclcpp::get_logger("SameFuzzyLogicController"), "A scoring function failed to prepare");
    }
  }

  try {
    dwb_msgs::msg::TrajectoryScore best = coreScoringAlgorithm(pose.pose, velocity, results);

    // Return Value
    nav_2d_msgs::msg::Twist2DStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.velocity = best.traj.velocity;

    // debrief stateful scoring functions
    for (dwb_core::TrajectoryCritic::Ptr & critic : critics_) {
      critic->debrief(cmd_vel.velocity);
    }

    lock.unlock();

    pub_->publishLocalPlan(pose.header, best.traj);
    pub_->publishCostGrid(costmap_ros_, critics_);

    return cmd_vel;
  } catch (const dwb_core::NoLegalTrajectoriesException & e) {
    nav_2d_msgs::msg::Twist2D empty_cmd;
    dwb_msgs::msg::Trajectory2D empty_traj;
    // debrief stateful scoring functions
    for (dwb_core::TrajectoryCritic::Ptr & critic : critics_) {
      critic->debrief(empty_cmd);
    }

    lock.unlock();

    pub_->publishLocalPlan(pose.header, empty_traj);
    pub_->publishCostGrid(costmap_ros_, critics_);

    throw nav2_core::NoValidControl(
            "Could not find a legal trajectory: " +
            std::string(e.what()));
  }
}

dwb_msgs::msg::TrajectoryScore
SameFuzzyLogicController::coreScoringAlgorithm(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  nav_2d_msgs::msg::Twist2D twist;
  dwb_msgs::msg::Trajectory2D traj;
  dwb_msgs::msg::TrajectoryScore best, worst;
  best.total = -1;
  worst.total = -1;
  dwb_core::IllegalTrajectoryTracker tracker;

  traj_generator_->startNewIteration(velocity);
  while (traj_generator_->hasMoreTwists()) {
    twist = traj_generator_->nextTwist();
    traj = traj_generator_->generateTrajectory(pose, velocity, twist);

    try {
      dwb_msgs::msg::TrajectoryScore score = scoreTrajectory(traj, best.total);
      tracker.addLegalTrajectory();
      if (results) {
        results->twists.push_back(score);
      }
      if (best.total < 0 || score.total < best.total) {
        best = score;
        if (results) {
          results->best_index = results->twists.size() - 1;
        }
      }
      if (worst.total < 0 || score.total > worst.total) {
        worst = score;
        if (results) {
          results->worst_index = results->twists.size() - 1;
        }
      }
    } catch (const dwb_core::IllegalTrajectoryException & e) {
      if (results) {
        dwb_msgs::msg::TrajectoryScore failed_score;
        failed_score.traj = traj;

        dwb_msgs::msg::CriticScore cs;
        cs.name = e.getCriticName();
        cs.raw_score = -1.0;
        failed_score.scores.push_back(cs);
        failed_score.total = -1.0;
        results->twists.push_back(failed_score);
      }
      tracker.addIllegalTrajectory(e);
    }
  }

  if (best.total < 0) {
    if (debug_trajectory_details_) {
      RCLCPP_ERROR(rclcpp::get_logger("SameFuzzyLogicController"), "%s", tracker.getMessage().c_str());
      for (auto const & x : tracker.getPercentages()) {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "SameFuzzyLogicController"), "%.2f: %10s/%s", x.second,
          x.first.first.c_str(), x.first.second.c_str());
      }
    }
    throw dwb_core::NoLegalTrajectoriesException(tracker);
  }

  return best;
}

dwb_msgs::msg::TrajectoryScore
SameFuzzyLogicController::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & traj,
  double best_score)
{
  dwb_msgs::msg::TrajectoryScore score;
  score.traj = traj;

  for (dwb_core::TrajectoryCritic::Ptr & critic : critics_) {
    dwb_msgs::msg::CriticScore cs;
    cs.name = critic->getName();
    cs.scale = critic->getScale();

    if (cs.scale == 0.0) {
      score.scores.push_back(cs);
      continue;
    }

    double critic_score = critic->scoreTrajectory(traj);
    cs.raw_score = critic_score;
    score.scores.push_back(cs);
    score.total += critic_score * cs.scale;
    if (short_circuit_trajectory_evaluation_ && best_score > 0 && score.total > best_score) {
      // since we keep adding positives, once we are worse than the best, we will stay worse
      break;
    }
  }

  return score;
}

nav_2d_msgs::msg::Path2D
SameFuzzyLogicController::transformGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  nav_2d_msgs::msg::Pose2DStamped robot_pose;
  if (!nav_2d_utils::transformPose(
      tf_, global_plan_.header.frame_id, pose,
      robot_pose, transform_tolerance_))
  {
    throw nav2_core::
          ControllerTFError("Unable to transform robot pose into global plan's frame");
  }

  // we'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;


  // If prune_plan is enabled (it is by default) then we want to restrict the
  // plan to distances within that range as well.
  double prune_dist = prune_distance_;

  // Set the maximum distance we'll include points before getting to the part
  // of the path where the robot is located (the start of the plan). Basically,
  // these are the points the robot has already passed.
  double transform_start_threshold;
  if (prune_plan_) {
    transform_start_threshold = std::min(dist_threshold, prune_dist);
  } else {
    transform_start_threshold = dist_threshold;
  }

  // Set the maximum distance we'll include points after the part of the part of
  // the plan near the robot (the end of the plan). This determines the amount
  // of the plan passed on to the critics
  double transform_end_threshold;
  if (shorten_transformed_plan_) {
    transform_end_threshold = std::min(dist_threshold, prune_dist);
  } else {
    transform_end_threshold = dist_threshold;
  }

  // Find the first pose in the global plan that's further than prune distance
  // from the robot using integrated distance
  auto prune_point = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), prune_dist);

  // Find the first pose in the plan (upto prune_point) that's less than transform_start_threshold
  // from the robot.
  auto transformation_begin = std::find_if(
    begin(global_plan_.poses), prune_point,
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose.pose, global_plan_pose) < transform_start_threshold;
    });

  // Find the first pose in the end of the plan that's further than transform_end_threshold
  // from the robot using integrated distance
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) {
      return euclidean_distance(pose, robot_pose.pose) > transform_end_threshold;
    });

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_2d_msgs::msg::Path2D transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Helper function for the transform below. Converts a pose2D from global
  // frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      nav_2d_msgs::msg::Pose2DStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.pose = global_plan_pose;
      nav_2d_utils::transformPose(
        tf_, transformed_plan.header.frame_id,
        stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose.pose;
    };

  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration.
  if (prune_plan_) {
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    pub_->publishGlobalPlan(global_plan_);
  }

  if (transformed_plan.poses.empty()) {
    throw nav2_core::InvalidPath("Resulting plan has 0 poses in it.");
  }
  return transformed_plan;
}

}  // namespace same_fuzzy_logic_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  same_fuzzy_logic_controller::SameFuzzyLogicController,
  nav2_core::Controller)
