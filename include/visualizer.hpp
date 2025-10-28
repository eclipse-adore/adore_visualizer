/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/
#pragma once
#include "adore_dynamics_conversions.hpp"
#include "adore_map_conversions.hpp"
#include "adore_math/angles.h"
#include "adore_ros2_msgs/msg/goal_point.hpp"
#include "adore_ros2_msgs/msg/node_status.hpp"
#include "adore_ros2_msgs/msg/traffic_prediction.hpp"
#include "adore_ros2_msgs/msg/vehicle_state_dynamic.hpp"
#include <adore_map/route.hpp>
#include <adore_math/point.h>
#include <adore_ros2_msgs/msg/infrastructure_info.hpp>
#include <adore_ros2_msgs/msg/traffic_participant_set.hpp>
#include <adore_ros2_msgs/msg/trajectory.hpp>
#include <foxglove_msgs/msg/geo_json.hpp>

#include "map_image_visualization.hpp"
#include "behavior_visualizer.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "building_visualization.hpp"
#include "visualization_primitives.hpp"
#include "visualizer_conversions.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nlohmann/json.hpp>
#include "adore_ros2_msgs/msg/node_status.hpp"
#include "adore_node_status.hpp"

using json = nlohmann::json;

namespace adore
{
namespace visualizer
{
class Visualizer : public rclcpp::Node
{
private:

  /* ---------- timing & TF -------------------------------------------------- */
  rclcpp::TimerBase::SharedPtr high_frequency_timer;
  rclcpp::TimerBase::SharedPtr low_frequency_timer;

  std::shared_ptr<tf2_ros::Buffer>               tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener>    tf_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  bool                                           have_initial_offset{ false };
  geometry_msgs::msg::TransformStamped           offset_tf;

  /* ---------- publishers ---------------------------------------------------- */
  using MarkerPublishers     = std::unordered_map<std::string, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr>;
  using TrajectoryPublishers = std::unordered_map<std::string, rclcpp::Publisher<adore_ros2_msgs::msg::TrajectoryTranspose>::SharedPtr>;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_grid_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr buildings_publisher;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr map_location_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr traffic_light_behavior_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vehicle_behavior_publisher;
  rclcpp::Publisher<foxglove_msgs::msg::GeoJSON>::SharedPtr route_visualization_publisher;
  rclcpp::Publisher<foxglove_msgs::msg::GeoJSON>::SharedPtr goal_visualization_publisher;

  MarkerPublishers                                           marker_publishers;
  TrajectoryPublishers                                       trajectory_publishers;

  /* ---------- subscriptions ------------------------------------------------- */
  rclcpp::Subscription<adore_ros2_msgs::msg::VehicleStateDynamic>::SharedPtr state_subscription;
  rclcpp::Subscription<adore_ros2_msgs::msg::InfrastructureInfo>::SharedPtr  infrastructure_info_subscription;
  rclcpp::Subscription<adore_ros2_msgs::msg::Route>::SharedPtr  route_subscriber;
  rclcpp::Subscription<adore_ros2_msgs::msg::NodeStatus>::SharedPtr  decision_maker_status_subscriber;
  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr>       dynamic_subscriptions;

  /* ---------- marker cache -------------------------------------------------- */
  std::unordered_map<std::string, MarkerArray> marker_cache;
  std::vector<buildings::Building> building_cache;

  /* ---------- configuration / state -------------------------------- */
  std::optional<math::Point2d> visualization_offset_center;
  math::Point2d                offset{};
  std::string                  assets_folder;
  GridTileCache                grid_tile_cache;
  TileKey                      latest_tile_idx{ -1, -1 };
  std::vector<std::string>     whitelist;
  std::string                  map_image_api_key;
  bool                         map_image_grayscale{ true };
  std::string                  ego_vehicle_3d_model_path;
  bool                         center_ego_vehicle{ true };
  std::optional<adore_ros2_msgs::msg::Route> ego_vehicle_route;
  std::optional<status::NodeStatus> ego_vehicle_decision_maker_status;

  /* ---------- dynamic‑subscription helpers --------------------------------- */
  template<typename MsgT>
  void update_dynamic_subscriptions( const std::string& desired_type );
  template<typename MsgT>
  void create_publisher_for( const std::string& topic_name );
  template<typename MsgT>
  void create_subscription_for( const std::string& topic_name );
  void update_all_dynamic_subscriptions();

  /* ---------- initialization ----------------------------------------------- */
  void load_parameters();
  void create_publishers();
  void create_subscribers();

  /* ---------- regular publishing helpers ----------------------------------- */
  void publish_map_image();
  void publish_buildings();
  void publish_map_location();
  void publish_map_route();
  void publish_markers();
  void publish_visualization_frame();
  void publish_traffic_light_behavior();
  void publish_vehicle_behavior();

  /* ---------- callbacks ----------------------------------------------------- */
  void vehicle_state_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg );
  void infrastructure_info_callback( const adore_ros2_msgs::msg::InfrastructureInfo& msg );
  void route_callback( const adore_ros2_msgs::msg::Route& msg );
  void decision_maker_status_callback( const adore_ros2_msgs::msg::NodeStatus& msg );
  void high_frequency_timer_callback();
  void low_frequency_timer_callback();


  /* ---------- misc. utilities ---------------------------------------------- */
  void change_frame( Marker& marker, const std::string& target_frame );
  bool should_subscribe_to_topic( const std::string& topic_name, const std::string& desired_type,
                                  const std::vector<std::string>& advertised_types ) const;

public:

  explicit Visualizer( const rclcpp::NodeOptions& options );
};

template<typename MsgT>
void
Visualizer::update_dynamic_subscriptions( const std::string& expected_msg_type )
{
  auto topic_names_and_types = get_topic_names_and_types();

  for( const auto& [topic_name, advertised_msg_types] : topic_names_and_types )
  {
    if( !should_subscribe_to_topic( topic_name, expected_msg_type, advertised_msg_types ) )
      continue;

    create_publisher_for<MsgT>( topic_name );
    create_subscription_for<MsgT>( topic_name );
  }
}

template<typename MsgT>
void
Visualizer::create_publisher_for( const std::string& topic_name )
{
  marker_publishers[topic_name] = create_publisher<visualization_msgs::msg::MarkerArray>( "viz" + topic_name, 10 );

  if constexpr( std::is_same_v<MsgT, adore_ros2_msgs::msg::Trajectory> )
  {
    trajectory_publishers[topic_name] = create_publisher<adore_ros2_msgs::msg::TrajectoryTranspose>( topic_name + "_transpose", 10 );
  }

  marker_cache[topic_name] = MarkerArray();
}

template<typename MsgT>
void
Visualizer::create_subscription_for( const std::string& topic_name )
{
  auto callback = [this, topic_name]( const MsgT& msg ) {
    // Skip conversion if nobody is subscribed to the marker topic
    const auto& pub_it = marker_publishers.find( topic_name );
    if( pub_it == marker_publishers.end() || pub_it->second->get_subscription_count() == 0 )
      return;

    auto marker_array = conversions::to_marker_array( msg );
    for( auto& marker : marker_array.markers )
    {
      marker.header.frame_id = msg.header.frame_id;
      change_frame( marker, "visualization_offset" );
    }

    marker_cache[topic_name] = marker_array;

    if constexpr( std::is_same_v<MsgT, adore_ros2_msgs::msg::Trajectory> )
    {
      trajectory_publishers[topic_name]->publish( dynamics::conversions::transpose( msg ) );
    }
  };

  dynamic_subscriptions[topic_name] = create_subscription<MsgT>( topic_name, 1, callback );
}

} // namespace visualizer
} // namespace adore
