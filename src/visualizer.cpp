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
#include "visualizer.hpp"
#include "adore_ros2_msgs/msg/goal_point.hpp"
#include "adore_ros2_msgs/msg/node_status.hpp"
#include "adore_ros2_msgs/msg/vehicle_state_dynamic.hpp"
#include <adore_map/lat_long_conversions.hpp>
#include <adore_map_conversions.hpp>
#include <adore_math/point.h>
#include <adore_node_status.hpp>
#include "behavior_visualizer.hpp"
#include "building_visualization.hpp"
#include "color_palette.hpp"
#include "foxglove_msgs/msg/geo_json.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_primitives.hpp"
#include <planning/optinlc_trajectory_optimizer.hpp>
#include <rclcpp/utilities.hpp>
#include <type_traits>

using namespace std::chrono_literals;

namespace adore
{
namespace visualizer
{

Visualizer::Visualizer(const rclcpp::NodeOptions & options) :
  Node( "visualizer_node" , options)
{
  load_parameters();
  create_publishers();
  create_subscribers();
}

void
Visualizer::load_parameters()
{
  declare_parameter( "asset folder", "" );
  get_parameter( "asset folder", assets_folder );

  declare_parameter( "whitelist", whitelist );
  get_parameter( "whitelist", whitelist );

  declare_parameter( "show_map_image", show_map_image );
  get_parameter( "show_map_image", show_map_image );

  if ( show_map_image )
  {
    declare_parameter( "map_image_api_key", map_image_api_key);
    get_parameter( "map_image_api_key", map_image_api_key);

    declare_parameter( "map_image_grayscale", map_image_grayscale);
    get_parameter( "map_image_grayscale", map_image_grayscale);
  }

  declare_parameter( "show_road_features", show_road_features);
  get_parameter( "show_road_features", show_road_features);

  offset_tf.header.frame_id         = "world";
  offset_tf.child_frame_id          = "visualization_offset";
  offset_tf.header.stamp            = now();

  offset_tf.transform.translation.x = declare_parameter("visualization_offset_x", 0.0);
  offset_tf.transform.translation.y = declare_parameter("visualization_offset_y", 0.0);
  offset_tf.transform.translation.z = 0.0;

  // This is to pre-cache images used in visualization due to the expensive runtime cost
  pre_cache_state_images();
  pre_cache_road_features();
}

void
Visualizer::pre_cache_state_images()
{
  std::optional<Image> traffic_light_gray_image = primitives::load_image(assets_folder + "/images/traffic_light_gray.png");
  if ( traffic_light_gray_image.has_value() )
  {
    pre_cached_images["traffic_light_gray"] = traffic_light_gray_image.value();
  }
  
  std::optional<Image> traffic_light_red_image = primitives::load_image(assets_folder + "/images/traffic_light_red.png");
  if ( traffic_light_red_image.has_value() )
  {
    pre_cached_images["traffic_light_red"] = traffic_light_red_image.value();
  }

  std::optional<Image> traffic_light_yellow_to_red_image = primitives::load_image(assets_folder + "/images/traffic_light_yellow_to_red.png");
  if ( traffic_light_yellow_to_red_image.has_value() )
  {
    pre_cached_images["traffic_light_yellow_to_red"] = traffic_light_yellow_to_red_image.value();
  }

  std::optional<Image> traffic_light_yellow_to_green_image = primitives::load_image(assets_folder + "/images/traffic_light_yellow_to_green.png");
  if ( traffic_light_yellow_to_green_image.has_value() )
  {
    pre_cached_images["traffic_light_yellow_to_green"] = traffic_light_yellow_to_green_image.value();
  }

  std::optional<Image> traffic_light_green_image = primitives::load_image(assets_folder + "/images/traffic_light_green.png");
  if ( traffic_light_green_image.has_value() )
  {
    pre_cached_images["traffic_light_green"] = traffic_light_green_image.value();
  }

  std::optional<Image> autonomous_car_on_road_image = primitives::load_image(assets_folder + "/images/autonomous_car_on_road.png");
  if ( autonomous_car_on_road_image .has_value() )
  {
    pre_cached_images["autonomous_car_on_road"] = autonomous_car_on_road_image.value();
  }

  std::optional<Image> autonomous_car_on_road_other_vehicle_close_image = primitives::load_image(assets_folder + "/images/autonomous_car_on_road_other_vehicle_close.png");
  if ( autonomous_car_on_road_other_vehicle_close_image .has_value() )
  {
    pre_cached_images["autonomous_car_on_road_other_vehicle_close"] = autonomous_car_on_road_other_vehicle_close_image.value();
  }

  std::optional<Image> autonomous_car_on_road_other_vehicle_middle_image = primitives::load_image(assets_folder + "/images/autonomous_car_on_road_other_vehicle_middle.png");
  if ( autonomous_car_on_road_other_vehicle_middle_image.has_value() )
  {
    pre_cached_images["autonomous_car_on_road_other_vehicle_middle"] = autonomous_car_on_road_other_vehicle_middle_image.value();
  }

  std::optional<Image> autonomous_car_on_road_other_vehicle_seen_image = primitives::load_image(assets_folder + "/images/autonomous_car_on_road_other_vehicle_seen.png");
  if ( autonomous_car_on_road_other_vehicle_seen_image .has_value() )
  {
    pre_cached_images["autonomous_car_on_road_other_vehicle_seen"] = autonomous_car_on_road_other_vehicle_seen_image.value();
  }
}

void
Visualizer::pre_cache_road_features()
{
  if ( !show_road_features )
    return;

  auto buildings = buildings::extract_buildings_from_road_features(assets_folder);
  pre_cached_road_features = buildings;
}

void
Visualizer::create_publishers()
{
  tf_broadcaster                   = std::make_shared<tf2_ros::TransformBroadcaster>( this );

  if ( show_map_image )
  {
    map_grid_publisher               = create_publisher<nav_msgs::msg::OccupancyGrid>( "map_grid", 1 );
  }

  if ( show_road_features )
  {
    road_feature_publisher = create_publisher<visualization_msgs::msg::MarkerArray>( "road_features", 1 );
  }

  // Most publishers are dynamic and generated later
}

void
Visualizer::create_subscribers()
{
  tf_buffer   = std::make_shared<tf2_ros::Buffer>( this->get_clock() );
  tf_listener = std::make_shared<tf2_ros::TransformListener>( *tf_buffer, this );

  high_frequency_timer = create_wall_timer( 100ms, std::bind( &Visualizer::high_frequency_timer_callback, this ) );
  low_frequency_timer  = create_wall_timer( 1000ms, std::bind( &Visualizer::low_frequency_timer_callback, this ) );
  update_all_dynamic_subscriptions();
}

void
Visualizer::update_all_dynamic_subscriptions()
{
  // @TODO, add a case for InfrastructureInfo
  update_dynamic_subscriptions<adore_ros2_msgs::msg::VehicleStateDynamic>( "adore_ros2_msgs/msg/VehicleStateDynamic" ); 
  update_dynamic_subscriptions<adore_ros2_msgs::msg::NodeStatus>( "adore_ros2_msgs/msg/NodeStatus" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::TrafficParticipantSet>( "adore_ros2_msgs/msg/TrafficParticipantSet" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::SafetyCorridor>( "adore_ros2_msgs/msg/SafetyCorridor" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::Map>( "adore_ros2_msgs/msg/Map" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::Route>( "adore_ros2_msgs/msg/Route" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::GoalPoint>( "adore_ros2_msgs/msg/GoalPoint" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::TrafficSignals>( "adore_ros2_msgs/msg/TrafficSignals" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::Waypoints>( "adore_ros2_msgs/msg/Waypoints" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::CautionZone>( "adore_ros2_msgs/msg/CautionZone" );
  // update_dynamic_subscriptions<adore_ros2_msgs::msg::Trajectory>( "adore_ros2_msgs/msg/Trajectory" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::VisualizableObject>( "adore_ros2_msgs/msg/VisualizableObject" );
}

void
Visualizer::high_frequency_timer_callback()
{
  publish_markers();
  publish_nav_sat_fix();
  publish_geo_json();

  publish_visualization_frame();
}

void
Visualizer::low_frequency_timer_callback()
{
  update_all_dynamic_subscriptions();

  publish_images();

  if ( show_map_image )
    publish_map_image();

  if ( show_road_features )
    publish_road_features();
}

void
Visualizer::publish_visualization_frame()
{

  offset_tf.header.stamp = now();
  tf_broadcaster->sendTransform( offset_tf );
}

void
Visualizer::publish_markers()
{
  for( const auto& [name, marker] : marker_cache )
  {
    if( marker_publishers.find( name ) != marker_publishers.end() )
    {
      marker_publishers[name]->publish( marker );
    }
  }
}

void
Visualizer::publish_nav_sat_fix()
{
  for( const auto& [name, fix] : nav_sat_fix_cache )
  {
    if( nav_sat_fix_publisher.find( name ) != nav_sat_fix_publisher.end() )
    {
      nav_sat_fix_publisher[name]->publish( fix );
    }
  }
}

void
Visualizer::publish_geo_json()
{
  for( const auto& [name, geo] : geo_json_cache )
  {
    if( geo_json_publisher.find( name ) != geo_json_publisher.end() )
    {
      geo_json_publisher[name]->publish( geo );
    }
  }
}

void
Visualizer::publish_images()
{
  for( const auto& [name, img] : images_traffic_cache )
  {
    if( image_traffic_light_publishers.find( name ) != image_traffic_light_publishers.end() )
    {
      image_traffic_light_publishers[name]->publish( img );
    }
  }

  for( const auto& [name, img] : images_behavior_cache )
  {
    if( image_behavior_publishers.find( name ) != image_behavior_publishers.end() )
    {
      image_behavior_publishers[name]->publish( img );
    }
  }
}

bool
Visualizer::should_subscribe_to_topic( const std::string& candidate_topic_name, const std::string& expected_msg_type,
                                       const std::vector<std::string>& advertised_msg_types ) const
{
  // Skip if this topic isn't whitelisted
  // A topic is whitelisted if it contains at least one of the allowed namespace prefixes
  if( !std::any_of( whitelist.begin(), whitelist.end(),
                    [&candidate_topic_name]( const std::string& ns ) { return candidate_topic_name.find( ns ) != std::string::npos; } ) )
    return false;

  // Skip if this topic doesn't advertise the type we're interested in
  if( std::find( advertised_msg_types.begin(), advertised_msg_types.end(), expected_msg_type ) == advertised_msg_types.end() )
    return false;

  // Skip if we've already subscribed to this topic
  if( dynamic_subscriptions.find( candidate_topic_name ) != dynamic_subscriptions.end() )
    return false;

  return true;
}

// @TODO, decide how to handle vehicle state and visualization
// void
// Visualizer::vehicle_state_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg )
// {
//   if( center_ego_vehicle )
//   {
//     // Add the latest odometry message to the buffer with its timestamp
//     auto latest_state           = dynamics::conversions::to_cpp_type( msg );
//     visualization_offset_center = { latest_state.x, latest_state.y };
//   }

//   // Convert the message to MarkerArray
//   // auto ego_marker_array                     = conversions::to_marker_array( msg );
//   // ego_marker_array.markers[0].mesh_resource = "http://localhost:8080/assets/3d_models/" + ego_vehicle_3d_model_path;

//   // Publish the MarkerArray
//   // marker_cache["ego_vehicle"] = ego_marker_array;
// }

void
Visualizer::publish_map_image()
{
  if ( !show_map_image )
    return;

  // Generate or retrieve cached PointCloud2
  auto index_and_tile = map_image::generate_occupancy_grid( offset_tf.transform.translation.x, offset_tf.transform.translation.y, assets_folder + "/maps/",
                                                            false, grid_tile_cache, map_image_api_key );

  if( latest_tile_idx != index_and_tile.first && map_grid_publisher->get_subscription_count() > 0 && index_and_tile.second.data.size() > 0 )
  {
    latest_tile_idx = index_and_tile.first;
    map_grid_publisher->publish( index_and_tile.second );
  }
}

void
Visualizer::publish_road_features()
{
  if ( !show_road_features )
    return;
  
  // @TODO, This whole function will need an overhaul in the future
  MarkerArray buildings_markers;
  
  std::vector<int> buildings_to_visualize_indexes = buildings::get_nearby_buildings(pre_cached_road_features, offset_tf.transform.translation.x, offset_tf.transform.translation.y, 200);

  for ( const int building_index : buildings_to_visualize_indexes )
  {
    const auto& building = pre_cached_road_features[building_index];

    double standard_building_height = 10.0;

    Marker building_marker = primitives::create_rectangle_marker(building.position_utm_x, building.position_utm_y, standard_building_height / 2.0, building.length / 2.0, building.width / 2.0, standard_building_height, building.orientation, "building", building_index, colors::white);
    building_marker.lifetime.sec = 20.0;
    building_marker.header.frame_id = "world";
    buildings_markers.markers.push_back(building_marker);
  }

  road_feature_publisher->publish( buildings_markers );
}

void
Visualizer::change_frame( visualization_msgs::msg::Marker& marker, const std::string& new_frame_id )
{
  try
  {
    geometry_msgs::msg::TransformStamped transform = tf_buffer->lookupTransform( new_frame_id, marker.header.frame_id, tf2::TimePointZero );
    primitives::transform_marker( marker, transform );
  }
  catch( const tf2::TransformException& ex )
  {
    RCLCPP_WARN( get_logger(), "Could not transform marker to new frame: %s", ex.what() );
    return;
  }
  marker.header.frame_id = new_frame_id;
}

} // namespace visualizer
} // namespace adore

int
main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );

  std::shared_ptr<adore::visualizer::Visualizer> node = std::make_shared<adore::visualizer::Visualizer>(rclcpp::NodeOptions{});
  rclcpp::spin( node );
  rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(adore::visualizer::Visualizer)
