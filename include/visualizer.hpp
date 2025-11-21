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
#include <unordered_map>
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

template <typename T>
concept CanConvertToMarker = requires(const T &t) {
    conversions::to_marker_array(t);  
};

template <typename T>
concept CanConvertToNavSatFix = requires(const T &t) {
    conversions::to_nav_sat_fix(t);  
};

template <typename T>
concept CanConvertToGeoJSON = requires(const T &t) {
    conversions::to_geo_json(t);  
};

template <typename T>
concept CanConvertToImage = requires(const T &t) {
    conversions::to_images(t);  
};

class Visualizer : public rclcpp::Node
{
private:

  /* ---------- initialization ----------------------------------------------- */
  void load_parameters();
  void create_publishers();
  void create_subscribers();

  void pre_cache_state_images(); // Since loading of files and images at runtime is expensive, some images shown in visualization will be cached in advance
  void pre_cache_road_features();

  std::unordered_map<std::string, Image> pre_cached_images; 
  std::vector<buildings::Building> pre_cached_road_features; // @TODO, Currently the only road feature is buildings, but expand on this in the future

  /* ---------- configuration / state -------------------------------- */
  std::vector<std::string>     whitelist;
  std::string                  assets_folder;
  bool show_map_image = false;
  bool show_road_features = false;

  std::string                  map_image_api_key;
  bool                         map_image_grayscale{ true };

  GridTileCache                grid_tile_cache;
  TileKey                      latest_tile_idx{ -1, -1 };

  /* ---------- timing & TF -------------------------------------------------- */
  rclcpp::TimerBase::SharedPtr high_frequency_timer;
  rclcpp::TimerBase::SharedPtr low_frequency_timer;

  std::shared_ptr<tf2_ros::Buffer>               tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener>    tf_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  bool                                           have_initial_offset{ false };
  geometry_msgs::msg::TransformStamped           offset_tf;

  /* ---------- callbacks ----------------------------------------------------- */
  void high_frequency_timer_callback();
  void low_frequency_timer_callback();

  /* ---------- regular publishing helpers ----------------------------------- */
  void publish_map_image();
  void publish_road_features();

  void publish_markers();
  void publish_nav_sat_fix();
  void publish_geo_json();
  void publish_images();

  void publish_visualization_frame();
  
  /* ---------- publishers ---------------------------------------------------- */
  using MarkerPublishers = std::unordered_map<std::string, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr>;
  using NavSatFixPublishers = std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr>;
  using GeoJSONPublishers = std::unordered_map<std::string, rclcpp::Publisher<foxglove_msgs::msg::GeoJSON>::SharedPtr>;
  using ImagePublishers = std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>;

  // @TODO, need to re-implement trajectory transpose behavior
  using TrajectoryPublishers = std::unordered_map<std::string, rclcpp::Publisher<adore_ros2_msgs::msg::TrajectoryTranspose>::SharedPtr>;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_grid_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr road_feature_publisher;

  MarkerPublishers                                           marker_publishers;
  NavSatFixPublishers                                        nav_sat_fix_publisher;
  GeoJSONPublishers                                          geo_json_publisher;
  TrajectoryPublishers                                       trajectory_publishers;
  ImagePublishers                                            image_traffic_light_publishers;
  ImagePublishers                                            image_behavior_publishers;

  /* ---------- subscriptions ------------------------------------------------- */
  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr>       dynamic_subscriptions;

  /* ---------- visualizer cache -------------------------------------------------- */
  std::unordered_map<std::string, MarkerArray> marker_cache;
  std::unordered_map<std::string, NavSatFix> nav_sat_fix_cache; 
  std::unordered_map<std::string, GeoJSON> geo_json_cache; 
  std::unordered_map<std::string, Image> images_traffic_cache; 
  std::unordered_map<std::string, Image> images_behavior_cache; 



  /* ---------- dynamic‑subscription helpers --------------------------------- */
  template<typename MsgT>
  void update_dynamic_subscriptions( const std::string& desired_type );

  template<typename MsgT>
  void create_publisher_for( const std::string& topic_name )
    requires( CanConvertToMarker<MsgT> || CanConvertToNavSatFix<MsgT> || CanConvertToGeoJSON<MsgT> || CanConvertToImage<MsgT> );

  template<typename MsgT>
  void create_subscription_for( const std::string& topic_name )
    requires( CanConvertToMarker<MsgT> || CanConvertToNavSatFix<MsgT> || CanConvertToGeoJSON<MsgT> || CanConvertToImage<MsgT> );

  template<typename MsgT>
  auto msg_covertion_lambda_callback( const std::string& topic_name )
    requires( CanConvertToMarker<MsgT> || CanConvertToNavSatFix<MsgT> || CanConvertToGeoJSON<MsgT> || CanConvertToImage<MsgT> );

  template<typename MsgT>
  void marker_callback_behavior(const std::string& topic_name, const MsgT& msg)
    requires( CanConvertToMarker<MsgT> );

  template<typename MsgT>
  void nav_sat_fix_callback_behavior(const std::string& topic_name, const MsgT& msg)
    requires( CanConvertToNavSatFix<MsgT> );

  template<typename MsgT>
  void geo_json_callback_behavior(const std::string& topic_name, const MsgT& msg)
    requires( CanConvertToGeoJSON<MsgT> );
    
  template<typename MsgT>
  void image_callback_behavior(const std::string& topic_name, const MsgT& msg)
    requires( CanConvertToImage<MsgT> );
    
  void update_all_dynamic_subscriptions();




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
    requires( CanConvertToMarker<MsgT> || CanConvertToNavSatFix<MsgT> || CanConvertToGeoJSON<MsgT> || CanConvertToImage<MsgT> )
{
  if constexpr( CanConvertToMarker<MsgT> )
  {
    marker_publishers[topic_name] = create_publisher<visualization_msgs::msg::MarkerArray>( "viz" + topic_name, 10 );
  }

  if constexpr( CanConvertToNavSatFix<MsgT> )
  {
    nav_sat_fix_publisher[topic_name] = create_publisher<sensor_msgs::msg::NavSatFix>( "nav_sat_fix_" + topic_name, 10 );
  }

  if constexpr( CanConvertToGeoJSON<MsgT> )
  {
    geo_json_publisher[topic_name] = create_publisher<foxglove_msgs::msg::GeoJSON>( "geo_json_" + topic_name, 10 );
  }

  if constexpr( CanConvertToImage<MsgT> )
  {
    image_traffic_light_publishers[topic_name] = create_publisher<sensor_msgs::msg::Image>( "image_traffic_light_" + topic_name, 10 );
    image_behavior_publishers[topic_name] = create_publisher<sensor_msgs::msg::Image>( "image_behavior_" + topic_name, 10 );
  }
}

template<typename MsgT>
void
Visualizer::create_subscription_for( const std::string& topic_name )
    requires( CanConvertToMarker<MsgT> || CanConvertToNavSatFix<MsgT> || CanConvertToGeoJSON<MsgT> || CanConvertToImage<MsgT> )
{
    auto callback = msg_covertion_lambda_callback<MsgT>(topic_name);
    dynamic_subscriptions[topic_name] = create_subscription<MsgT>( topic_name, 1, callback );
}

template<typename MsgT>
auto
Visualizer::msg_covertion_lambda_callback( const std::string& topic_name )
    requires( CanConvertToMarker<MsgT> || CanConvertToNavSatFix<MsgT> || CanConvertToGeoJSON<MsgT> || CanConvertToImage<MsgT> )
{
  return [this, topic_name]( const MsgT& msg ) {

    // This setup allows for multiple kinds of convertions to happen on the same topic
    
    if constexpr( CanConvertToMarker<MsgT>)
    {
      marker_callback_behavior<MsgT>(topic_name, msg);
    }

    if constexpr( CanConvertToNavSatFix<MsgT>)
    {
      nav_sat_fix_callback_behavior<MsgT>(topic_name, msg);
    }

    if constexpr( CanConvertToGeoJSON<MsgT>)
    {
      geo_json_callback_behavior<MsgT>(topic_name, msg);
    }

    if constexpr( CanConvertToImage<MsgT>)
    {
      image_callback_behavior<MsgT>(topic_name, msg);
    }
    
  };
}

template<typename MsgT>
void
Visualizer::marker_callback_behavior( const std::string& topic_name, const MsgT& msg )
  requires( CanConvertToMarker<MsgT> )
{
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
}

template<typename MsgT>
void
Visualizer::nav_sat_fix_callback_behavior(const std::string& topic_name, const MsgT& msg)
  requires( CanConvertToNavSatFix<MsgT> )
{
    const auto& pub_it = nav_sat_fix_publisher.find( topic_name );
    if( pub_it == nav_sat_fix_publisher.end() || pub_it->second->get_subscription_count() == 0 )
      return;
  
    auto nav_sat_fix = conversions::to_nav_sat_fix( msg );
    nav_sat_fix_cache[topic_name] = nav_sat_fix;
}

template<typename MsgT>
void
Visualizer::geo_json_callback_behavior(const std::string& topic_name, const MsgT& msg)
  requires( CanConvertToGeoJSON<MsgT> )
{
    const auto& pub_it = geo_json_publisher.find( topic_name );
    if( pub_it == geo_json_publisher.end() || pub_it->second->get_subscription_count() == 0 )
      return;

    auto geo_json = conversions::to_geo_json( msg );
    geo_json_cache[topic_name] = geo_json;
}

template<typename MsgT>
void
Visualizer::image_callback_behavior(const std::string& topic_name, const MsgT& msg)
  requires( CanConvertToImage<MsgT> )
{
  // @TODO, add this for the other publisher
  const auto& pub_it = image_traffic_light_publishers.find( topic_name );
  if( pub_it == image_traffic_light_publishers.end() || pub_it->second->get_subscription_count() == 0 )
    return;

  auto image_names = conversions::to_images( msg );

  for ( const std::string& name : image_names )
  {
    if ( !pre_cached_images.contains( name ) )
      continue;
    
    Image image = pre_cached_images[name];

    if ( name.find( "traffic" ) != std::string::npos ) // Check if it is an image for a traffic light
    {
      images_traffic_cache[topic_name] = image;
    }

    if ( name.find( "autonomous" ) != std::string::npos ) // Check if it is an image for a behavior
    {
      images_behavior_cache[topic_name] = image;
    }
  }
}

} // namespace visualizer
} // namespace adore
