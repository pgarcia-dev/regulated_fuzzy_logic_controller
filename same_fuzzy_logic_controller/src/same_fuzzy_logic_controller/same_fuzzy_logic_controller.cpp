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
#include <tf2/transform_datatypes.h>

#include "fl/Headers.h"
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

using namespace nav2_costmap_2d;  // NOLINT



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

  // Handles global path transformations
  path_handler_ = std::make_unique<same_fuzzy_logic_controller::PathHandler>(
    tf2::durationFromSec(0.1), tf_, costmap_ros_);//params_->transform_tolerance. no 0.1
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
  path_handler_->setPlan(path);




 ////////RCLCPP_INFO(logger_, "*****************next paths's position - x: %f , y: %f, z: %f, vector size: %ld", path.poses.at(0).pose.position.x, path.poses.at(0).pose.position.y, path.poses.at(0).pose.position.z, path.poses.size());
 ///////RCLCPP_INFO(logger_, "*****************next paths's position back - x: %f , y: %f", path.poses.back().pose.position.x, path.poses.back().pose.position.y);
  //RCLCPP_INFO(logger_, "*****************next paths's position begin - x: %f , y: %f", path.poses.begin()->pose.position.x, path.poses.begin()->pose.position.y);
// RCLCPP_INFO(logger_, "next paths's orientation - x: %f , y: %f, z: %f, w: %f", path.poses.at(0).pose.orientation.x, path.poses.at(0).pose.orientation.y, path.poses.at(0).pose.orientation.z, path.poses.at(0).pose.orientation.w);
 // if (next_waypoint_x_ != path.poses.begin()->pose.position.x || next_waypoint_y_ != path.poses.begin()->pose.position.y)
  //  RCLCPP_INFO(logger_, "*** ########### NEW NEXT WAYPOINT ############");

  //next_waypoint_x_ = path.poses.begin()->pose.position.x;
  //next_waypoint_y_ = path.poses.begin()->pose.position.y;

  //RCLCPP_INFO(logger_, "*** frame reference path: %s",path.header.frame_id.c_str());
 // RCLCPP_INFO(logger_, "*** frame reference next wp: %s",path.header.frame_id);

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
 /// RCLCPP_INFO(logger_, "***********2");

  //RCLCPP_INFO(logger_, "*** r pose reference frame: %s",pose.header.frame_id.c_str());

  //RCLCPP_INFO(logger_, "*** robot's position - x: %f , y: %f", pose.pose.position.x, pose.pose.position.y);
  //RCLCPP_INFO(logger_, "robot's orientation - x: %f , y: %f, z: %f, w: %f", pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

 // RCLCPP_INFO(logger_, "*** robot's position - x: %f , y: %f", pose.pose.position.x, pose.pose.position.y);
  //RCLCPP_INFO(logger_, "*** next paths's position - x: %f , y: %f", next_waypoint_x_, next_waypoint_y_);

  tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw); //***************** getYaw instead
  RCLCPP_INFO(logger_, "*** robot's orientation - roll: %f , pitch: %f, yaw: %f", roll, pitch, yaw);

    // Transform path to robot base frame
  auto transformed_plan = path_handler_->transformGlobalPlan(
    pose, 5);//********5 aprox
  //global_path_pub_->publish(transformed_plan);//*********



  double dx = transformed_plan.poses.begin()->pose.position.x;// - pose.pose.position.x; //*******xq -?
  double dy = transformed_plan.poses.begin()->pose.position.y;// - pose.pose.position.y; 

  double heading = std::atan2(dy, dx); //+ yaw;//*************- yaw
  RCLCPP_INFO(logger_, "***222 heading: %f- next paths's position - x: %f , y: %f", heading, transformed_plan.poses.begin()->pose.position.x, transformed_plan.poses.begin()->pose.position.y);
 // heading -= 3;
 // RCLCPP_INFO(logger_, "***33 heading: %f- next paths's position - x: %f , y: %f", heading, transformed_plan.poses.begin()->pose.position.x, transformed_plan.poses.begin()->pose.position.y);

 // double heading2 = atan2(dx, dy);
 // double heading = angle;// - yaw;    //***********


  //and here I just make sure my angle is between minus pi and pi! 
  //if (heading > M_PI)
    //heading -= (2*M_PI);
  //if (heading <= -M_PI)
    //heading += 2*M_PI;


  


 // RCLCPP_INFO(logger_, "*** real heading %f",heading);
  
 // if (heading > 0) //////////////// temporal until using TF2
  //  heading -= M_PI;
  //else
   // heading += M_PI;

  

  //if (heading > 0.5 || heading < -0.5) //------------------------------
  //  heading = 0;


  
  //RCLCPP_INFO(logger_, "***1 robot's position - x: %f , y: %f, z: %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  






  using namespace fl;

  static const float INPUT_NL_MIN = -3.15;
  static const float INPUT_NL_MED = -3.0;
  static const float INPUT_NL_MAX = -2.8;
  static const float INPUT_NM_MIN = -2.8;
  static const float INPUT_NM_MED = -1.9;
  static const float INPUT_NM_MAX = -1.1;
  static const float INPUT_N_MIN = -1.1;
  static const float INPUT_N_MED = -0.9;
  static const float INPUT_N_MAX = -0.6;
  static const float INPUT_NS_MIN = -0.6;
  static const float INPUT_NS_MED = -0.5;
  static const float INPUT_NS_MAX = -0.4;
  static const float INPUT_ZN_MIN = -0.4;
  static const float INPUT_ZN_MED = -0.25;
  static const float INPUT_ZN_MAX = -0.1;
  static const float INPUT_Z_MIN = -0.1; 
  static const float INPUT_Z_MED = 0; //-------------
  static const float INPUT_Z_MAX = 0.1;
  static const float INPUT_ZP_MIN = 0.1;
  static const float INPUT_ZP_MED = 0.25;
  static const float INPUT_ZP_MAX = 0.4;
  static const float INPUT_PS_MIN = 0.4;
  static const float INPUT_PS_MED = 0.5;
  static const float INPUT_PS_MAX = 0.6;
  static const float INPUT_P_MIN = 0.6;
  static const float INPUT_P_MED = 0.9;
  static const float INPUT_P_MAX = 1.1;
  static const float INPUT_PM_MIN = 1.1;
  static const float INPUT_PM_MED = 1.9;
  static const float INPUT_PM_MAX = 2.8;
  static const float INPUT_PL_MIN = 2.8;
  static const float INPUT_PL_MED = 3.0;
  static const float INPUT_PL_MAX = 3.15;

  static const float OUTPUT_ANG_NL_MIN = -1;
  static const float OUTPUT_ANG_NL_MED = -0.8;
  static const float OUTPUT_ANG_NL_MAX = -0.6;
  static const float OUTPUT_ANG_NM_MIN = -0.6;
  static const float OUTPUT_ANG_NM_MED = -0.45;
  static const float OUTPUT_ANG_NM_MAX = -0.31;
  static const float OUTPUT_ANG_N_MIN = -0.31;
  static const float OUTPUT_ANG_N_MED = -0.25;
  static const float OUTPUT_ANG_N_MAX = -0.19;
  static const float OUTPUT_ANG_NS_MIN = -0.19;
  static const float OUTPUT_ANG_NS_MED = -0.15;
  static const float OUTPUT_ANG_NS_MAX = -0.1;
  static const float OUTPUT_ANG_ZN_MIN = -0.1;
  static const float OUTPUT_ANG_ZN_MED = -0.05;
  static const float OUTPUT_ANG_ZN_MAX = -0.025;
  static const float OUTPUT_ANG_Z_MIN = -0.025; 
  static const float OUTPUT_ANG_Z_MED = 0; //-------------
  static const float OUTPUT_ANG_Z_MAX = 0.025;
  static const float OUTPUT_ANG_ZP_MIN = 0.025;
  static const float OUTPUT_ANG_ZP_MED = 0.05;
  static const float OUTPUT_ANG_ZP_MAX = 0.1;
  static const float OUTPUT_ANG_PS_MIN = 0.1;
  static const float OUTPUT_ANG_PS_MED = 0.15;
  static const float OUTPUT_ANG_PS_MAX = 0.19;
  static const float OUTPUT_ANG_P_MIN = 0.19;
  static const float OUTPUT_ANG_P_MED = 0.25;
  static const float OUTPUT_ANG_P_MAX = 0.31;
  static const float OUTPUT_ANG_PM_MIN = 0.31;
  static const float OUTPUT_ANG_PM_MED = 0.45;
  static const float OUTPUT_ANG_PM_MAX = 0.6;
  static const float OUTPUT_ANG_PL_MIN = 0.6;
  static const float OUTPUT_ANG_PL_MED = 0.8;
  static const float OUTPUT_ANG_PL_MAX = 1;

  static const float OUTPUT_LIN_S_MIN = 0;
  static const float OUTPUT_LIN_S_MED = 0.11;
  static const float OUTPUT_LIN_S_MAX = 0.22;
  static const float OUTPUT_LIN_M_MIN = 0.19;
  static const float OUTPUT_LIN_M_MED = 0.2;
  static const float OUTPUT_LIN_M_MAX = 0.4;
  static const float OUTPUT_LIN_L_MIN = 0.35;
  static const float OUTPUT_LIN_L_MED = 0.45;
  static const float OUTPUT_LIN_L_MAX = 0.55;
  static const float OUTPUT_LIN_VL_MIN = 0.5; 
  static const float OUTPUT_LIN_VL_MED = 0.6; 
  static const float OUTPUT_LIN_VL_MAX = 0.7;

  Engine *engine = new Engine;
  engine->setName("SFLC");
  engine->setDescription("");

  InputVariable* Uao_gtg = new InputVariable;
  Uao_gtg->setName("Uao_gtg");
  Uao_gtg->setDescription("");
  Uao_gtg->setEnabled(true);
  Uao_gtg->setRange(-3.8,3.8); 
  Uao_gtg->setLockValueInRange(false);
  Uao_gtg->addTerm(new Triangle("NL", INPUT_NL_MIN, INPUT_NL_MED, INPUT_NL_MAX));
  Uao_gtg->addTerm(new Triangle("NM", INPUT_NM_MIN, INPUT_NM_MED, INPUT_NM_MAX));
  Uao_gtg->addTerm(new Triangle("N",  INPUT_N_MIN,  INPUT_N_MED,  INPUT_N_MAX));
  Uao_gtg->addTerm(new Triangle("NS", INPUT_NS_MIN, INPUT_NS_MED, INPUT_NS_MAX));
  Uao_gtg->addTerm(new Triangle("ZN", INPUT_ZN_MIN, INPUT_ZN_MED, INPUT_ZN_MAX));
  Uao_gtg->addTerm(new Triangle("Z",  INPUT_Z_MIN,  INPUT_Z_MED,  INPUT_Z_MAX)); //----------
  Uao_gtg->addTerm(new Triangle("ZP", INPUT_ZP_MIN, INPUT_ZP_MED, INPUT_ZP_MAX));
  Uao_gtg->addTerm(new Triangle("PS", INPUT_PS_MIN, INPUT_PS_MED, INPUT_PS_MAX));
  Uao_gtg->addTerm(new Triangle("P",  INPUT_P_MIN,  INPUT_P_MED,  INPUT_P_MAX));
  Uao_gtg->addTerm(new Triangle("PM", INPUT_PM_MIN, INPUT_PM_MED, INPUT_PM_MAX));
  Uao_gtg->addTerm(new Triangle("PL", INPUT_PL_MIN, INPUT_PL_MED, INPUT_PL_MAX));
  engine->addInputVariable(Uao_gtg);

  OutputVariable* linear_velocity = new OutputVariable;
  linear_velocity->setName("linear_velocity");
  linear_velocity->setDescription("");
  linear_velocity->setEnabled(true);
  linear_velocity->setRange(0, 0.7);
  linear_velocity->setLockValueInRange(false);
  linear_velocity->setAggregation(new Maximum);
  linear_velocity->setDefuzzifier(new LargestOfMaximum);
  linear_velocity->setDefaultValue(fl::nan);
  linear_velocity->setLockPreviousValue(false);
  linear_velocity->addTerm(new Triangle("S",  OUTPUT_LIN_S_MIN, OUTPUT_LIN_S_MED, OUTPUT_LIN_S_MAX));
  linear_velocity->addTerm(new Triangle("M",  OUTPUT_LIN_M_MIN, OUTPUT_LIN_M_MED, OUTPUT_LIN_M_MAX));
  linear_velocity->addTerm(new Triangle("L",  OUTPUT_LIN_L_MIN, OUTPUT_LIN_L_MED, OUTPUT_LIN_L_MAX));
  linear_velocity->addTerm(new Triangle("VL", OUTPUT_LIN_VL_MIN, OUTPUT_LIN_VL_MED, OUTPUT_LIN_VL_MAX));
  engine->addOutputVariable(linear_velocity);

  OutputVariable* angular_velocity = new OutputVariable;
  angular_velocity->setName("angular_velocity");
  angular_velocity->setDescription("");
  angular_velocity->setEnabled(true);
  angular_velocity->setRange(-1,1);
  angular_velocity->setLockValueInRange(false);
  angular_velocity->setAggregation(new Maximum);
  angular_velocity->setDefuzzifier(new LargestOfMaximum);
  angular_velocity->setDefaultValue(fl::nan);
  angular_velocity->setLockPreviousValue(false);
  angular_velocity->addTerm(new Triangle("NL", OUTPUT_ANG_NL_MIN, OUTPUT_ANG_NL_MED, OUTPUT_ANG_NL_MAX));
  angular_velocity->addTerm(new Triangle("NM", OUTPUT_ANG_NM_MIN, OUTPUT_ANG_NM_MED, OUTPUT_ANG_NM_MAX));
  angular_velocity->addTerm(new Triangle("N",  OUTPUT_ANG_N_MIN,  OUTPUT_ANG_N_MED,  OUTPUT_ANG_N_MAX));
  angular_velocity->addTerm(new Triangle("NS", OUTPUT_ANG_NS_MIN, OUTPUT_ANG_NS_MED, OUTPUT_ANG_NS_MAX));
  angular_velocity->addTerm(new Triangle("ZN", OUTPUT_ANG_ZN_MIN, OUTPUT_ANG_ZN_MED, OUTPUT_ANG_ZN_MAX));
  angular_velocity->addTerm(new Triangle("Z",  OUTPUT_ANG_Z_MIN,  OUTPUT_ANG_Z_MED,  OUTPUT_ANG_Z_MAX)); //----------
  angular_velocity->addTerm(new Triangle("ZP", OUTPUT_ANG_ZP_MIN, OUTPUT_ANG_ZP_MED, OUTPUT_ANG_ZP_MAX));
  angular_velocity->addTerm(new Triangle("PS", OUTPUT_ANG_PS_MIN, OUTPUT_ANG_PS_MED, OUTPUT_ANG_PS_MAX));
  angular_velocity->addTerm(new Triangle("P",  OUTPUT_ANG_P_MIN,  OUTPUT_ANG_P_MED,  OUTPUT_ANG_P_MAX));
  angular_velocity->addTerm(new Triangle("PM", OUTPUT_ANG_PM_MIN, OUTPUT_ANG_PM_MED, OUTPUT_ANG_PM_MAX));
  angular_velocity->addTerm(new Triangle("PL", OUTPUT_ANG_PL_MIN, OUTPUT_ANG_PL_MED, OUTPUT_ANG_PL_MAX));
  engine->addOutputVariable(angular_velocity);

  RuleBlock* ruleBlock = new RuleBlock;
  ruleBlock->setName("");
  ruleBlock->setDescription("");
  ruleBlock->setEnabled(true);
  //ruleBlock->setConjunction(fl::null);
  //ruleBlock->setDisjunction(fl::null);
  ruleBlock->setEnabled(true);
  ruleBlock->setConjunction(new Minimum);
  ruleBlock->setDisjunction(new Maximum);
  ruleBlock->setImplication(new Minimum);
  //  ruleBlock->setImplication(new AlgebraicProduct);
  ruleBlock->setActivation(new General);
  
  ruleBlock->addRule(Rule::parse("if Uao_gtg is NL then linear_velocity is S and angular_velocity is NL", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is NM then linear_velocity is S and angular_velocity is NM", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is N then linear_velocity is S and angular_velocity is N", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is NS then linear_velocity is M and angular_velocity is NS", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is ZN then linear_velocity is L and angular_velocity is ZN", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is Z then linear_velocity is VL and angular_velocity is Z", engine));/////////////
  ruleBlock->addRule(Rule::parse("if Uao_gtg is ZP then linear_velocity is L and angular_velocity is ZP", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is PS then linear_velocity is M and angular_velocity is PS", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is P then linear_velocity is S and angular_velocity is P", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is PM then linear_velocity is S and angular_velocity is PM", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is PL then linear_velocity is S and angular_velocity is PL", engine));
  engine->addRuleBlock(ruleBlock);
 
 /**
   ruleBlock->addRule(Rule::parse("if Uao_gtg is NL then linear_velocity is S and angular_velocity is Z", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is NM then linear_velocity is S and angular_velocity is Z", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is N then linear_velocity is S and angular_velocity is Z", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is NS then linear_velocity is M and angular_velocity is Z", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is Z then linear_velocity is VL and angular_velocity is Z", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is ZP then linear_velocity is L and angular_velocity is Z", engine)); /////////////
  ruleBlock->addRule(Rule::parse("if Uao_gtg is PS then linear_velocity is M and angular_velocity is Z", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is PS then linear_velocity is M and angular_velocity is Z", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is P then linear_velocity is S and angular_velocity is Z", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is PM then linear_velocity is S and angular_velocity is Z", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is PL then linear_velocity is S and angular_velocity is Z", engine));
   

  ruleBlock->addRule(Rule::parse("if Uao_gtg is NL then linear_velocity is S and angular_velocity is PL", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is NM then linear_velocity is S and angular_velocity is PM", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is N then linear_velocity is S and angular_velocity is P", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is NS then linear_velocity is M and angular_velocity is PS", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is ZN then linear_velocity is L and angular_velocity is ZP", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is Z then linear_velocity is VL and angular_velocity is Z", engine));/////////////
  ruleBlock->addRule(Rule::parse("if Uao_gtg is ZP then linear_velocity is L and angular_velocity is ZN", engine)); 
  ruleBlock->addRule(Rule::parse("if Uao_gtg is PS then linear_velocity is M and angular_velocity is NS", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is P then linear_velocity is S and angular_velocity is N", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is PM then linear_velocity is S and angular_velocity is NM", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is PL then linear_velocity is S and angular_velocity is NL", engine));
  **/

/**
  ruleBlock->addRule(Rule::parse("if Uao_gtg is NL then linear_velocity is S and angular_velocity is PL", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is NM then linear_velocity is S and angular_velocity is PM", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is N then linear_velocity is S and angular_velocity is P", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is NS then linear_velocity is M and angular_velocity is PS", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is ZN then linear_velocity is L and angular_velocity is ZP", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is Z then linear_velocity is VL and angular_velocity is Z", engine));/////////////
  ruleBlock->addRule(Rule::parse("if Uao_gtg is ZP then linear_velocity is L and angular_velocity is ZN", engine)); 
  ruleBlock->addRule(Rule::parse("if Uao_gtg is PS then linear_velocity is M and angular_velocity is NS", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is P then linear_velocity is S and angular_velocity is N", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is PM then linear_velocity is S and angular_velocity is NM", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is PL then linear_velocity is S and angular_velocity is NL", engine));
**/

  engine->addRuleBlock(ruleBlock);

  Uao_gtg->setValue(heading);
  engine->process();

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;  
  cmd_vel.twist.linear.x = linear_velocity->getValue();
  cmd_vel.twist.angular.z = angular_velocity->getValue();
 // RCLCPP_INFO(logger_, "*** robot's position - x: %f , y: %f", pose.pose.position.x, pose.pose.position.y);
  //RCLCPP_INFO(logger_, "***2 input heading:%f, output linear:%f, output angular: %f  ",heading, cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);

  return cmd_vel;
  

/**
  scalar input_angle = -3.14;
  FL_LOG(Op::str(input_angle) << ",, " << Uao_gtg->range())
  for (int i = 0; i < 50; ++i){
    input_angle += 0.125;
    Uao_gtg->setValue(input_angle);
    engine->process();
    FL_LOG( "input = " << Op::str(input_angle) << " => " << "output = linear:" << Op::str(linear_velocity->getValue()) << ", angular:" << Op::str(angular_velocity->getValue()));
  }
**/




























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
