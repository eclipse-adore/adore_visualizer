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
  Node( "visualizer_node" ),
  state_buffer( 10.0 )
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

  declare_parameter( "visualization_offset_x", 606456.0 );
  declare_parameter( "visualization_offset_y", 5797319.0 );
  get_parameter( "visualization_offset_x", offset.x );
  get_parameter( "visualization_offset_y", offset.y );


  create_publishers();
  create_subscribers();
}

void
Visualizer::create_publishers()
{
  visualisation_transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>( this );
  map_cloud_publisher                 = create_publisher<sensor_msgs::msg::PointCloud2>( "map_cloud", 1 );
  marker_publishers["driven_path"]    = create_publisher<visualization_msgs::msg::MarkerArray>( "visualize_driven_path", 1 );
  marker_publishers["ego_vehicle"]    = create_publisher<visualization_msgs::msg::MarkerArray>( "visualize_ego_vehicle", 1 );
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
  update_dynamic_subscriptions<bob_perception_msgs::msg::TrackedOrientedBoxV2xArray>(
    "bob_perception_msgs/msg/TrackedOrientedBoxV2xArray" );
}

void
Visualizer::create_subscribers()
{
  subscriber_vehicle_state_dynamic = create_subscription<adore_ros2_msgs::msg::VehicleStateDynamic>(
    "vehicle_state/dynamic", 1, std::bind( &Visualizer::vehicle_state_dynamic_callback, this, std::placeholders::_1 ) );

  update_all_dynamic_subscriptions();

  main_timer = create_wall_timer( 100ms, std::bind( &Visualizer::timer_callback, this ) );
}

void
Visualizer::timer_callback()
{
  for( const auto& [name, marker] : markers_to_publish )
  {
    if( marker_publishers.find( name ) != marker_publishers.end() )
    {
      marker_publishers[name]->publish( marker );
    }
  }

  update_all_dynamic_subscriptions();

  if( !latest_state )
    return;

  // Generate or retrieve cached PointCloud2
  auto index_and_tile = map_image::generate_pointcloud2( offset, *latest_state, maps_folder, false, tile_cache, map_image_api_key );

  if( latest_tile_index != index_and_tile.first && map_cloud_publisher->get_subscription_count() > 0
      && index_and_tile.second.data.size() > 0 )
  {
    latest_tile_index = index_and_tile.first;
    map_cloud_publisher->publish( index_and_tile.second );
  }
}

void
Visualizer::vehicle_state_dynamic_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg )
{
  // Add the latest odometry message to the buffer with its timestamp
  latest_state = dynamics::conversions::to_cpp_type( msg );
  state_buffer.add( msg );

  // Convert the buffer to a marker array representing the driven path
  auto driven_path_array            = conversions::to_marker_array( state_buffer, offset );
  markers_to_publish["driven_path"] = driven_path_array;

  // Convert the message to MarkerArray
  auto ego_marker_array                     = conversions::to_marker_array( msg, offset );
  ego_marker_array.markers[0].mesh_resource = "http://localhost:8080/assets/3d_models/" + ego_vehicle_3d_model_path;


  // Publish the MarkerArray
  markers_to_publish["ego_vehicle"] = ego_marker_array;

  publish_visualization_offset();
}

void
Visualizer::publish_visualization_offset()
{

  geometry_msgs::msg::TransformStamped viz_transform;

  viz_transform.header.stamp    = this->get_clock()->now();
  viz_transform.header.frame_id = "world";
  viz_transform.child_frame_id  = "visualization_offset";

  viz_transform.transform.translation.x = offset.x;
  viz_transform.transform.translation.y = offset.y;
  viz_transform.transform.translation.z = 0;

  tf2::Quaternion q;
  q.setRPY( 0, 0, 0 );
  viz_transform.transform.rotation.x = q.x();
  viz_transform.transform.rotation.y = q.y();
  viz_transform.transform.rotation.z = q.z();
  viz_transform.transform.rotation.w = q.w();

  visualisation_transform_broadcaster->sendTransform( viz_transform );
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