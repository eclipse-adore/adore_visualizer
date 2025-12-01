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
#include "adore_ros2_msgs/msg/traffic_prediction.hpp"
#include "adore_ros2_msgs/msg/vehicle_state_dynamic.hpp"
#include <adore_map/route.hpp>
#include <adore_math/point.h>
#include <adore_ros2_msgs/msg/infrastructure_info.hpp>
#include <adore_ros2_msgs/msg/traffic_participant_set.hpp>
#include <adore_ros2_msgs/msg/trajectory.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "visualizable_traits.hpp"
#include "visualization_primitives.hpp"
#include "visualizer_conversions.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster;
  std::shared_ptr<tf2_ros::Buffer>                     tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener>          tf_listener;
  geometry_msgs::msg::TransformStamped                 offset_tf;

  /* ---------- publishers ---------------------------------------------------- */
  using MarkerPublishers     = std::unordered_map<std::string, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr>;
  using TrajectoryPublishers = std::unordered_map<std::string, rclcpp::Publisher<adore_ros2_msgs::msg::TrajectoryTranspose>::SharedPtr>;
  using NavSatFixPublishers  = std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr>;
  using GeoJSONPublishers    = std::unordered_map<std::string, rclcpp::Publisher<foxglove_msgs::msg::GeoJSON>::SharedPtr>;

  MarkerPublishers     marker_publishers;
  TrajectoryPublishers trajectory_publishers;
  NavSatFixPublishers  nav_sat_fix_publishers;
  GeoJSONPublishers    geo_json_publishers;

  /* ---------- subscriptions ------------------------------------------------- */
  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> dynamic_subscriptions;

  /* ---------- marker cache -------------------------------------------------- */
  std::unordered_map<std::string, MarkerArray> marker_cache;

  /* ---------- configuration / state -------------------------------- */
  std::optional<math::Point2d> visualization_offset_center;
  std::vector<std::string>     whitelist;

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
  void create_subscribers();

  /* ---------- regular publishing helpers ----------------------------------- */
  void publish_markers();

  /* ---------- callbacks ----------------------------------------------------- */
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
  // MarkerArray publisher only if MsgT can be converted
  if constexpr( has_marker_array_conversion<MsgT> )
  {
    marker_publishers[topic_name] = create_publisher<visualization_msgs::msg::MarkerArray>( "viz" + topic_name, 10 );
    marker_cache[topic_name]      = MarkerArray();
  }

  // TrajectoryTranspose publisher only if MsgT has transpose conversion
  if constexpr( has_trajectory_transpose_conversion<MsgT> )
  {
    trajectory_publishers[topic_name] = create_publisher<adore_ros2_msgs::msg::TrajectoryTranspose>( topic_name + "_transpose", 10 );
  }

  if constexpr( has_nav_sat_fix_conversion<MsgT> )
  {
    using NavSatMsg                    = typename nav_sat_fix_conversion<MsgT>::result_type;
    nav_sat_fix_publishers[topic_name] = create_publisher<NavSatMsg>( topic_name + "_nav_sat_fix", 10 );
  }

  if constexpr( has_geo_json_conversion<MsgT> )
  {
    using GeoJSONMsg                = typename geo_json_conversion<MsgT>::result_type;
    geo_json_publishers[topic_name] = create_publisher<GeoJSONMsg>( topic_name + "_geo_json", 10 );
  }
}

template<typename MsgT>
void
Visualizer::create_subscription_for( const std::string& topic_name )
{
  auto callback = [this, topic_name]( const MsgT& msg ) {
    // MarkerArray branch (only compiled if conversion exists)
    if constexpr( has_marker_array_conversion<MsgT> )
    {
      const auto& pub_it = marker_publishers.find( topic_name );
      if( pub_it == marker_publishers.end() || pub_it->second->get_subscription_count() == 0 )
        return;

      auto marker_array = marker_array_conversion<MsgT>::convert( msg );
      for( auto& marker : marker_array.markers )
      {
        marker.header.frame_id = msg.header.frame_id;
        change_frame( marker, "visualization_offset" );
      }
      marker_publishers[topic_name]->publish( marker_array );

      // marker_cache[topic_name] = std::move( marker_array );
    }

    // TrajectoryTranspose branch (only compiled if conversion exists)
    if constexpr( has_trajectory_transpose_conversion<MsgT> )
    {
      trajectory_publishers[topic_name]->publish( trajectory_transpose_conversion<MsgT>::convert( msg ) );
    }

    // NavSatFix branch (only compiled if conversion exists)
    if constexpr( has_nav_sat_fix_conversion<MsgT> )
    {
      nav_sat_fix_publishers[topic_name]->publish( nav_sat_fix_conversion<MsgT>::convert( msg ) );
    }

    // GeoJSON branch (only compiled if conversion exists)
    if constexpr( has_geo_json_conversion<MsgT> )
    {
      geo_json_publishers[topic_name]->publish( geo_json_conversion<MsgT>::convert( msg ) );
    }
  };

  dynamic_subscriptions[topic_name] = create_subscription<MsgT>( topic_name, 1, callback );
}


} // namespace visualizer
} // namespace adore
