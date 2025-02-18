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
Visualizer::update_trajectory_subscriptions()
{
  auto topic_names_and_types = get_topic_names_and_types();

  for( const auto& topic_pair : topic_names_and_types )
  {
    const std::string&              topic_name = topic_pair.first;
    const std::vector<std::string>& types      = topic_pair.second;

    // Check whether this topic advertises our expected type.
    if( std::find( types.begin(), types.end(), "adore_ros2_msgs/msg/Trajectory" ) != types.end() )
    {
      // If we haven't already subscribed to this topic, do so.
      if( trajectory_subscriptions.find( topic_name ) == trajectory_subscriptions.end() )
      {
        trajectory_publishers[topic_name] = create_publisher<adore_ros2_msgs::msg::TrajectoryTranspose>( topic_name + "_transpose", 1 );
        marker_publishers[topic_name]     = create_publisher<visualization_msgs::msg::MarkerArray>( "visualize_" + topic_name, 1 );
        markers_to_publish[topic_name]    = MarkerArray(); // Initialize empty marker array until we get somethign to publish

        auto subscription = create_subscription<adore_ros2_msgs::msg::Trajectory>(
          topic_name, 1, [this, topic_name]( const adore_ros2_msgs::msg::Trajectory& msg ) {
            markers_to_publish[topic_name] = conversions::to_marker_array( msg, offset );
            trajectory_publishers[topic_name]->publish( dynamics::conversions::transpose( msg ) );
          } );

        trajectory_subscriptions[topic_name] = subscription;

        RCLCPP_INFO( get_logger(), "Dynamically subscribed to topic: %s", topic_name.c_str() );
      }
    }
  }
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

  update_trajectory_subscriptions();
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
    else
    {
      RCLCPP_ERROR( get_logger(), "No publisher found for %s", name.c_str() );
    }
  }
  update_all_dynamic_subscriptions();

  if( !latest_state )
    return;

  auto cloud            = map_image::generate_pointcloud2( offset, latest_state.value(), maps_folder, false );
  cloud.header.frame_id = "visualization_offset";
  map_cloud_publisher->publish( cloud );
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
  auto odom_marker_array = conversions::to_marker_array( msg, offset );

  // Publish the MarkerArray
  markers_to_publish["ego_vehicle"] = odom_marker_array;


  if( !first_state.has_value() )
  {
    first_state = latest_state;
    offset.x    = msg.x;
    offset.y    = msg.y;
  }

  publish_visualization_offset();
}

void
Visualizer::publish_visualization_offset()
{

  geometry_msgs::msg::TransformStamped viz_transform;

  viz_transform.header.stamp    = this->get_clock()->now();
  viz_transform.header.frame_id = "world";
  viz_transform.child_frame_id  = "visualization_offset";

  viz_transform.transform.translation.x = 0;
  viz_transform.transform.translation.y = 0;
  viz_transform.transform.translation.z = 0;

  tf2::Quaternion q;
  q.setRPY( 0, 0, 0 );
  viz_transform.transform.rotation.x = q.x();
  viz_transform.transform.rotation.y = q.y();
  viz_transform.transform.rotation.z = q.z();
  viz_transform.transform.rotation.w = q.w();

  geometry_msgs::msg::TransformStamped ego_viz_transform;

  ego_viz_transform.header.frame_id = "visualization_offset";
  ego_viz_transform.child_frame_id  = "ego_follow_visualization";

  ego_viz_transform.transform.translation.x = latest_state->x - offset.x;
  ego_viz_transform.transform.translation.y = latest_state->y - offset.y;

  tf2::Quaternion q2;
  q2.setRPY( 0, 0, latest_state->yaw_angle );
  ego_viz_transform.transform.rotation.x = q2.x();
  ego_viz_transform.transform.rotation.y = q2.y();
  ego_viz_transform.transform.rotation.z = q2.z();
  ego_viz_transform.transform.rotation.w = q2.w();

  visualisation_transform_broadcaster->sendTransform( viz_transform );
  visualisation_transform_broadcaster->sendTransform( ego_viz_transform );
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