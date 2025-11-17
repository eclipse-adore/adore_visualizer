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
using namespace std::chrono_literals;

namespace adore
{
namespace visualizer
{

Visualizer::Visualizer( const rclcpp::NodeOptions& options ) :
  Node( "visualizer_node", options )
{
  load_parameters();
  create_publishers();
  create_subscribers();
}

void
Visualizer::load_parameters()
{
  maps_folder                    = declare_parameter<std::string>( "asset folder", "" );
  ego_vehicle_3d_model_path      = declare_parameter<std::string>( "ego_vehicle_3d_model_path", "low_poly_ngc_model.dae" );
  whitelist                      = declare_parameter<std::vector<std::string>>( "whitelist", whitelist );
  map_image_api_key              = declare_parameter<std::string>( "map_image_api_key", "" );
  map_image_grayscale            = declare_parameter<bool>( "map_image_grayscale", true );
  visualization_offset_center    = math::Point2d( 0.0, 0.0 );
  visualization_offset_center->x = declare_parameter<double>( "visualization_offset_x", 0.0 );
  visualization_offset_center->y = declare_parameter<double>( "visualization_offset_y", 0.0 );

  offset_tf.header.frame_id         = "world";
  offset_tf.child_frame_id          = "visualization_offset";
  offset_tf.header.stamp            = now();
  offset_tf.transform.translation.x = visualization_offset_center->x;
  offset_tf.transform.translation.y = visualization_offset_center->y;
  offset_tf.transform.translation.z = 0.0;
}

void
Visualizer::create_publishers()
{
  tf_broadcaster                   = std::make_shared<tf2_ros::TransformBroadcaster>( this );
  map_grid_publisher               = create_publisher<nav_msgs::msg::OccupancyGrid>( "map_grid", 1 );
  marker_publishers["ego_vehicle"] = create_publisher<visualization_msgs::msg::MarkerArray>( "visualize_ego_vehicle", 1 );
}

void
Visualizer::update_all_dynamic_subscriptions()
{
  update_dynamic_subscriptions<adore_ros2_msgs::msg::TrafficParticipantSet>( "adore_ros2_msgs/msg/TrafficParticipantSet" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::SafetyCorridor>( "adore_ros2_msgs/msg/SafetyCorridor" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::Map>( "adore_ros2_msgs/msg/Map" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::Route>( "adore_ros2_msgs/msg/Route" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::GoalPoint>( "adore_ros2_msgs/msg/GoalPoint" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::TrafficSignals>( "adore_ros2_msgs/msg/TrafficSignals" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::Waypoints>( "adore_ros2_msgs/msg/Waypoints" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::CautionZone>( "adore_ros2_msgs/msg/CautionZone" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::Trajectory>( "adore_ros2_msgs/msg/Trajectory" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::VisualizableObject>( "adore_ros2_msgs/msg/VisualizableObject" );
  update_dynamic_subscriptions<adore_ros2_msgs::msg::VehicleStateDynamic>( "adore_ros2_msgs/msg/VehicleStateDynamic" );
}

void
Visualizer::create_subscribers()
{
  tf_buffer            = std::make_shared<tf2_ros::Buffer>( this->get_clock() );
  tf_listener          = std::make_shared<tf2_ros::TransformListener>( *tf_buffer, this );
  high_frequency_timer = create_wall_timer( 100ms, std::bind( &Visualizer::high_frequency_timer_callback, this ) );
  low_frequency_timer  = create_wall_timer( 1000ms, std::bind( &Visualizer::low_frequency_timer_callback, this ) );
  update_all_dynamic_subscriptions();
}

void
Visualizer::high_frequency_timer_callback()
{
  publish_visualization_frame();
  publish_markers();
}

void
Visualizer::low_frequency_timer_callback()
{
  update_all_dynamic_subscriptions();
  publish_map_image();
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

void
Visualizer::publish_map_image()
{
  if( !visualization_offset_center )
    return;

  // Generate or retrieve cached PointCloud2
  auto index_and_tile = map_image::generate_occupancy_grid( visualization_offset_center->x, visualization_offset_center->y, maps_folder,
                                                            false, grid_tile_cache, map_image_api_key );

  if( latest_tile_idx != index_and_tile.first && map_grid_publisher->get_subscription_count() > 0 && index_and_tile.second.data.size() > 0 )
  {
    latest_tile_idx = index_and_tile.first;
    map_grid_publisher->publish( index_and_tile.second );
  }
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

  std::shared_ptr<adore::visualizer::Visualizer> node = std::make_shared<adore::visualizer::Visualizer>( rclcpp::NodeOptions{} );
  rclcpp::spin( node );
  rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::visualizer::Visualizer )
