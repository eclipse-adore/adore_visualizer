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
#include "adore_dynamics_conversions.hpp"
#include "adore_map_conversions.hpp"
#include "adore_math/angles.h"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "visualizer_conversions.hpp"
#include "adore_ros2_msgs/msg/infrastructure_info.hpp"
#include "adore_ros2_msgs/msg/traffic_prediction.hpp"
#include "adore_ros2_msgs/msg/vehicle_state_dynamic.hpp"
#include <adore_ros2_msgs/msg/traffic_participant_set.hpp>
#include <adore_ros2_msgs/msg/trajectory.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include "visualization_primitives.hpp"
#include "map_image_visualization.hpp"

namespace adore
{
namespace visualizer
{


class InfrastructureVisualizer: public rclcpp::Node
{
public:

  InfrastructureVisualizer();

private:

  // Timers & TF
  rclcpp::TimerBase::SharedPtr main_timer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> visualisation_offset_transform_broadcaster;


  // Subscriber/Publisher Creation Functions
  void load_parameters();
  void create_subscribers();
  void create_publishers();

  void timer_callback();

  // Subscriptions
  rclcpp::Subscription<adore_ros2_msgs::msg::InfrastructureInfo>::SharedPtr subscriber_infrastructure_info;
  rclcpp::Subscription<adore_ros2_msgs::msg::Map>::SharedPtr subscriber_local_map;
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr subscriber_traffic_participant_set;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_infrastructure_marker;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_local_map_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_validation_area;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_traffic_participant_markers;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_map_cloud;

  void publish_visualization_offset();

  // State
  std::optional<adore_ros2_msgs::msg::InfrastructureInfo> latest_infrastructure_info;
  std::optional<adore_ros2_msgs::msg::Map> latest_local_map;
  std::optional<adore_ros2_msgs::msg::TrafficParticipantSet> latest_traffic_participant_set;
  bool visualize_infrastructure = false;
  bool visualize_local_map = false;
  bool visualize_validity_area = false;
  bool visualize_map_image = false;
  bool visualize_traffic_participants = false;

  std::string maps_folder;
  TileCache tile_cache;
  TileKey latest_tile_index = { -1, -1 };
  std::string map_image_api_key;
  bool map_image_grayscale = true;

  // Cllbacks
  void infrastructure_info_callback(const adore_ros2_msgs::msg::InfrastructureInfo& msg);
  void local_map_callback(const adore_ros2_msgs::msg::Map& msg);
  void traffic_participant_callback(const adore_ros2_msgs::msg::TrafficParticipantSet& msg);
};

} // namespace visualizer
} // namespace adore
