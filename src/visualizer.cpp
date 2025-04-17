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

#include "visualizer.hpp"
using namespace std::chrono_literals;

namespace adore
{
namespace visualizer
{

Visualizer::Visualizer() :
  Node( "visualizer_node" )
{
  load_parameters();
  create_publishers();
  create_subscribers();
}

void
Visualizer::load_parameters()
{
  declare_parameter( "asset folder", "" );
  get_parameter( "asset folder", maps_folder );

  declare_parameter( "ego_vehicle_3d_model_path", "low_poly_ngc_model.dae" );
  get_parameter( "ego_vehicle_3d_model_path", ego_vehicle_3d_model_path );

  declare_parameter( "whitelist", whitelist );
  get_parameter( "whitelist", whitelist );

  declare_parameter( "map_image_api_key", "" );
  get_parameter( "map_image_api_key", map_image_api_key );

  declare_parameter( "map_image_grayscale", true );
  get_parameter( "map_image_grayscale", map_image_grayscale );

  declare_parameter( "center_ego_vehicle", true );
  get_parameter( "center_ego_vehicle", center_ego_vehicle );
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
}

void
Visualizer::create_subscribers()
{
  tf_buffer   = std::make_shared<tf2_ros::Buffer>( this->get_clock() );
  tf_listener = std::make_shared<tf2_ros::TransformListener>( *tf_buffer, this );

  state_subscription = create_subscription<adore_ros2_msgs::msg::VehicleStateDynamic>( "vehicle_state/dynamic", 1,
                                                                                       std::bind( &Visualizer::vehicle_state_callback, this,
                                                                                                  std::placeholders::_1 ) );

  infrastructure_info_subscription = create_subscription<adore_ros2_msgs::msg::InfrastructureInfo>(
    "infrastructure_info", 1, std::bind( &Visualizer::infrastructure_info_callback, this, std::placeholders::_1 ) );

  high_frequency_timer = create_wall_timer( 100ms, std::bind( &Visualizer::high_frequency_timer_callback, this ) );
  low_frequency_timer  = create_wall_timer( 1000ms, std::bind( &Visualizer::low_frequency_timer_callback, this ) );
  update_all_dynamic_subscriptions();
}

void
Visualizer::high_frequency_timer_callback()
{
  publish_markers();
  if( !map_center )
    return;
  publish_visualization_frame();
}

void
Visualizer::low_frequency_timer_callback()
{
  update_all_dynamic_subscriptions();
  if( !map_center )
    return;
  publish_map_image();
}

void
Visualizer::publish_visualization_frame()
{
  // initialize visualization transform
  if( !have_initial_offset )
  {
    have_initial_offset               = true;
    offset_tf.header.frame_id         = "world";
    offset_tf.child_frame_id          = "visualization_offset";
    offset_tf.header.stamp            = now();
    offset_tf.transform.translation.x = map_center->x;
    offset_tf.transform.translation.y = map_center->y;
    offset_tf.transform.translation.z = 0.0;
  }
  // Publish the offset transform
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
Visualizer::vehicle_state_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg )
{
  if( center_ego_vehicle )
  {
    // Add the latest odometry message to the buffer with its timestamp
    auto latest_state = dynamics::conversions::to_cpp_type( msg );
    map_center        = { latest_state.x, latest_state.y };
  }

  // Convert the message to MarkerArray
  auto ego_marker_array                     = conversions::to_marker_array( msg );
  ego_marker_array.markers[0].mesh_resource = "http://localhost:8080/assets/3d_models/" + ego_vehicle_3d_model_path;

  // Publish the MarkerArray
  marker_cache["ego_vehicle"] = ego_marker_array;
}

void
Visualizer::infrastructure_info_callback( const adore_ros2_msgs::msg::InfrastructureInfo& msg )
{
  if( !center_ego_vehicle )
  {
    map_center = { msg.position_x, msg.position_y };
  }
}

void
Visualizer::publish_map_image()
{
  if( !map_center )
    return;

  // Generate or retrieve cached PointCloud2
  auto index_and_tile = map_image::generate_occupancy_grid( map_center->x, map_center->y, maps_folder, false, grid_tile_cache,
                                                            map_image_api_key );

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

  std::shared_ptr<adore::visualizer::Visualizer> node = std::make_shared<adore::visualizer::Visualizer>();
  rclcpp::spin( node );
  rclcpp::shutdown();
}