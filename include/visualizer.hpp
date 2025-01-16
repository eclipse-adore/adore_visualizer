/********************************************************************************
 * Copyright (C) 2024-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 ********************************************************************************/
#pragma once
#include "adore_dynamics_conversions.hpp"
#include "adore_map_conversions.hpp"
#include "adore_math/angles.h"
#include "adore_ros2_msgs/msg/traffic_prediction.hpp"
#include "adore_ros2_msgs/msg/vehicle_state_dynamic.hpp"
#include <adore_ros2_msgs/msg/traffic_participant_set.hpp>
#include <adore_ros2_msgs/msg/trajectory.hpp>

#include "map_image_visualization.hpp"
#include "state_buffer.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualizer_conversions.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace adore
{
namespace visualizer
{

class Visualizer : public rclcpp::Node
{
private:


  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr subscriber_traffic_participants;
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr subscriber_ignored_participants;
  rclcpp::Subscription<adore_ros2_msgs::msg::VehicleStateDynamic>::SharedPtr   subscriber_vehicle_state_dynamic;

  rclcpp::Subscription<adore_ros2_msgs::msg::Trajectory>::SharedPtr        subscriber_decision;
  rclcpp::Subscription<adore_ros2_msgs::msg::Trajectory>::SharedPtr        subscriber_trajectory;
  rclcpp::Subscription<adore_ros2_msgs::msg::Trajectory>::SharedPtr        subscriber_controller_trajectory;
  rclcpp::Subscription<adore_ros2_msgs::msg::SafetyCorridor>::SharedPtr    subscriber_safety_corridor;
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficPrediction>::SharedPtr subscriber_traffic_prediction;
  rclcpp::Subscription<adore_ros2_msgs::msg::Route>::SharedPtr             subscriber_route;
  rclcpp::Subscription<adore_ros2_msgs::msg::Map>::SharedPtr               subscriber_local_map;
  rclcpp::Subscription<adore_ros2_msgs::msg::GoalPoint>::SharedPtr         subscriber_goal;
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficSignals>::SharedPtr    subscriber_traffic_signals;

  rclcpp::TimerBase::SharedPtr main_timer;

  std::unique_ptr<tf2_ros::TransformBroadcaster> visualisation_transform_broadcaster;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;

  // Map to store publishers for MarkerArray for different topics
  std::unordered_map<std::string, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr>      marker_publishers;
  std::unordered_map<std::string, rclcpp::Publisher<adore_ros2_msgs::msg::TrajectoryTranspose>::SharedPtr> trajectory_publishers;

  // Map to store MarkerArrays corresponding to different topics
  std::unordered_map<std::string, visualization_msgs::msg::MarkerArray> marker_array_map;

  // callback functions
  void traffic_participants_callback( const adore_ros2_msgs::msg::TrafficParticipantSet& msg );
  void ignored_participants_callback( const adore_ros2_msgs::msg::TrafficParticipantSet& msg );
  void vehicle_state_dynamic_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg );
  void decision_callback( const adore_ros2_msgs::msg::Trajectory& msg );
  void trajectory_callback( const adore_ros2_msgs::msg::Trajectory& msg );
  void controller_trajectory_callback( const adore_ros2_msgs::msg::Trajectory& msg );
  void safety_corridor_callback( const adore_ros2_msgs::msg::SafetyCorridor& msg );
  void traffic_prediction_callback( const adore_ros2_msgs::msg::TrafficPrediction& msg );
  void map_callback( const adore_ros2_msgs::msg::Map& msg );
  void route_callback( const adore_ros2_msgs::msg::Route& msg );
  void goal_callback( const adore_ros2_msgs::msg::GoalPoint& msg );
  void traffic_signals_callback( const adore_ros2_msgs::msg::TrafficSignals& msg );
  void timer_callback();

  StateBuffer state_buffer;
  double      keep_odometry_time = 20; // seconds

  std::optional<dynamics::VehicleStateDynamic> first_state;
  std::optional<dynamics::VehicleStateDynamic> latest_state;
  void                                         publish_visualization_offset();

  Offset offset;

  std::string maps_folder;

  std::vector<std::string>                     marker_names;
  std::unordered_map<std::string, MarkerArray> markers_to_publish;

  std::vector<std::string> visualizing_trajectory_names;

public:

  Visualizer();

  // Function to create and store subscribers for various topics
  void create_subscribers();

  // Function to create publishers for various topics
  void create_publishers();
};

} // namespace visualizer
} // namespace adore
