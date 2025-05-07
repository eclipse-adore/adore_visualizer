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
 *    Mikkel Skov Maarssø
 ********************************************************************************/
#pragma once
#include <adore_math/point.h>
#include "adore_dynamics_conversions.hpp"
#include "adore_map_conversions.hpp"
#include "adore_math/angles.h"
#include <string>
#include "adore_ros2_msgs/msg/caution_zone.hpp"
#include "adore_ros2_msgs/msg/goal_point.hpp"
#include "adore_ros2_msgs/msg/map.hpp"
#include "adore_ros2_msgs/msg/traffic_participant.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "adore_ros2_msgs/msg/traffic_prediction.hpp"
#include "adore_ros2_msgs/msg/trajectory.hpp"
#include "adore_ros2_msgs/msg/vehicle_state_dynamic.hpp"
#include <adore_ros2_msgs/msg/traffic_participant_set.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "adore_ros2_msgs/msg/waypoints.hpp"
#include "visualizer_conversions.hpp"
#include <adore_ros2_msgs/msg/trajectory.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include "map_image_visualization.hpp"

namespace adore
{
namespace visualizer
{


class VehicleVisualizer : public rclcpp::Node
{
private:

  // Timers & TF
  rclcpp::TimerBase::SharedPtr                   main_timer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> vehicle_transform_broadcaster;
  std::unique_ptr<tf2_ros::TransformBroadcaster> visualisation_offset_transform_broadcaster;

  // Subscriber/Publisher Creation Functions
  void create_subscribers();
  void create_publishers();

  // Subscriptions
  rclcpp::Subscription<adore_ros2_msgs::msg::VehicleStateDynamic>::SharedPtr subscriber_vehicle_state_dynamic;
  rclcpp::Subscription<adore_ros2_msgs::msg::Trajectory>::SharedPtr subscriber_trajectory;
  rclcpp::Subscription<adore_ros2_msgs::msg::Map>::SharedPtr subscriber_local_map;
  rclcpp::Subscription<adore_ros2_msgs::msg::GoalPoint>::SharedPtr subscriber_goal_point;
  rclcpp::Subscription<adore_ros2_msgs::msg::Route>::SharedPtr subscriber_route;
  rclcpp::Subscription<adore_ros2_msgs::msg::CautionZone>::SharedPtr subscriber_caution_zone;
  rclcpp::Subscription<adore_ros2_msgs::msg::Waypoints>::SharedPtr subscriber_remote_operation_waypoints;
  rclcpp::Subscription<adore_ros2_msgs::msg::Trajectory>::SharedPtr subscriber_remote_operation_trajectory;
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr subscriber_traffic_participants;
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr subscriber_ignored_traffic_participants;
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr subscriber_traffic_participants_predicted_trajectories;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_vehicle_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_trajectory_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_state_markers ;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_local_map_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_goal_point_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_route_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_caution_zones;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_traffic_participant_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_ignored_traffic_participant_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_traffic_participant_predicted_trajectories;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_map_cloud;
 
  // State
  std::optional<Offset> visualization_offset;
  std::optional<adore_ros2_msgs::msg::VehicleStateDynamic> latest_vehicle_state_dynamic;
  std::optional<adore_ros2_msgs::msg::Trajectory> latest_trajectory;
  std::optional<adore_ros2_msgs::msg::Map> latest_local_map;
  std::optional<adore_ros2_msgs::msg::GoalPoint> latest_goal_point;
  std::optional<adore_ros2_msgs::msg::Route> latest_route;
  std::optional<adore_ros2_msgs::msg::CautionZone> latest_caution_zone;
  std::optional<adore_ros2_msgs::msg::TrafficParticipantSet> latest_traffic_participant_set;
  std::optional<adore_ros2_msgs::msg::TrafficParticipantSet> latest_ignored_traffic_participant_set;
  std::optional<adore_ros2_msgs::msg::TrafficParticipantSet> latest_traffic_participants_set_with_predictions;

  std::string maps_folder;
  TileCache tile_cache;
  TileKey latest_tile_index = { -1, -1 };
  std::string map_image_api_key;
  bool map_image_grayscale = true;
  rclcpp::Time current_time;
  rclcpp::Time last_update_time;
  bool visualize_vehicle = false;
  bool visualize_trajectory = false;
  bool visualize_state = false;
  bool visualize_local_map = false;
  bool visualize_goal_point = false;
  bool visualize_route = false;
  bool visualize_map_image = false;
  bool visualize_traffic_participants = false;
  bool visualize_traffic_participants_predicted_trajectories = false;
  bool visualize_ignored_traffic_participants = false;
  bool visualize_remote_operations = false;

  // callback functions
  void timer_callback();
  void vehicle_state_dynamic_callback(const adore_ros2_msgs::msg::VehicleStateDynamic& msg);
  void planned_trajectory_callback(const adore_ros2_msgs::msg::Trajectory& msg);
  void local_map_callback(const adore_ros2_msgs::msg::Map& msg);
  void caution_zone_callback(const adore_ros2_msgs::msg::CautionZone& msg);
  void route_callback(const adore_ros2_msgs::msg::Route& msg);
  void goal_point_callback(const adore_ros2_msgs::msg::GoalPoint& msg);
  void traffic_participant_set_callback(const adore_ros2_msgs::msg::TrafficParticipantSet& msg);
  void ignored_traffic_participant_set_callback(const adore_ros2_msgs::msg::TrafficParticipantSet& msg);
  void publish_visualization_offset();
  void traffic_participant_set_with_predictions_callback(const adore_ros2_msgs::msg::TrafficParticipantSet& msg);

public:

  VehicleVisualizer();
};

} // namespace visualizer
} // namespace adore
