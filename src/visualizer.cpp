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
  state_buffer( 20.0 )
{

  marker_names = { "borders",
                   "local_map",
                   "safety_corridor",
                   "ignored_participants",
                   "traffic_prediction",
                   "route",
                   "traffic_participants",
                   "driven_path",
                   "trajectory_decision",
                   "planned_trajectory",
                   "controller_trajectory",
                   "ego_vehicle",
                   "goal" };

  visualizing_trajectory_names = { "trajectory_decision", "planned_trajectory", "controller_trajectory" };

  declare_parameter( "asset folder", "" );
  get_parameter( "asset folder", maps_folder );
  create_publishers();
  create_subscribers();
}

void
Visualizer::create_publishers()
{


  // Loop over marker names to create publishers
  for( const auto& name : marker_names )
  {
    std::string topic_name   = "visualization/" + name;
    marker_publishers[name]  = this->create_publisher<visualization_msgs::msg::MarkerArray>( topic_name, 10 );
    markers_to_publish[name] = MarkerArray();
  }

  visualisation_transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>( this );

  map_publisher = create_publisher<nav_msgs::msg::OccupancyGrid>( "map", 10 );

  for( const auto& name : visualizing_trajectory_names )
  {
    std::string topic_name      = "vis_traj/" + name;
    trajectory_publishers[name] = create_publisher<adore_ros2_msgs::msg::TrajectoryTranspose>( topic_name, 1 );
  }
}

void
Visualizer::create_subscribers()
{
  // Subscriber for traffic participants
  subscriber_traffic_participants = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>(
    "traffic_participants", 1, std::bind( &Visualizer::traffic_participants_callback, this, std::placeholders::_1 ) );

  // Subscriber for traffic participants
  subscriber_ignored_participants = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>(
    "ignored_participants", 1, std::bind( &Visualizer::ignored_participants_callback, this, std::placeholders::_1 ) );


  subscriber_vehicle_state_dynamic = create_subscription<adore_ros2_msgs::msg::VehicleStateDynamic>(
    "vehicle_state/dynamic", 1, std::bind( &Visualizer::vehicle_state_dynamic_callback, this, std::placeholders::_1 ) );


  subscriber_decision = create_subscription<adore_ros2_msgs::msg::Trajectory>( "trajectory_decision", 1,
                                                                               std::bind( &Visualizer::decision_callback, this,
                                                                                          std::placeholders::_1 ) );

  subscriber_trajectory = create_subscription<adore_ros2_msgs::msg::Trajectory>( "planned_trajectory", 1,
                                                                                 std::bind( &Visualizer::trajectory_callback, this,
                                                                                            std::placeholders::_1 ) );

  subscriber_controller_trajectory = create_subscription<adore_ros2_msgs::msg::Trajectory>(
    "controller_trajectory", 1, std::bind( &Visualizer::controller_trajectory_callback, this, std::placeholders::_1 ) );

  subscriber_safety_corridor = create_subscription<adore_ros2_msgs::msg::SafetyCorridor>( "safety_corridor", 1,
                                                                                          std::bind( &Visualizer::safety_corridor_callback,
                                                                                                     this, std::placeholders::_1 ) );

  subscriber_traffic_prediction = create_subscription<adore_ros2_msgs::msg::TrafficPrediction>(
    "traffic_prediction", 1, std::bind( &Visualizer::traffic_prediction_callback, this, std::placeholders::_1 ) );

  subscriber_borders = create_subscription<adore_ros2_msgs::msg::BorderArray>( "local_borders", 1,
                                                                               std::bind( &Visualizer::borders_callback, this,
                                                                                          std::placeholders::_1 ) );

  subscriber_local_map = create_subscription<adore_ros2_msgs::msg::Map>( "local_map", 1,
                                                                         std::bind( &Visualizer::map_callback, this,
                                                                                    std::placeholders::_1 ) );

  subscriber_route = create_subscription<adore_ros2_msgs::msg::Route>( "route", 1,
                                                                       std::bind( &Visualizer::route_callback, this,
                                                                                  std::placeholders::_1 ) );

  subscriber_goal = create_subscription<adore_ros2_msgs::msg::GoalPoint>( "mission/goal_position", 1,
                                                                          std::bind( &Visualizer::goal_callback, this,
                                                                                     std::placeholders::_1 ) );

  main_timer = create_wall_timer( 100ms, std::bind( &Visualizer::timer_callback, this ) );
}

void
Visualizer::timer_callback()
{
  for( const auto& name : marker_names )
  {
    marker_publishers[name]->publish( markers_to_publish[name] );
  }
  if( !latest_state )
    return;

  auto occupancy_grid            = map_image::generate_occupancy_grid( offset, latest_state.value(), maps_folder, false );
  occupancy_grid.header.frame_id = "visualization_offset";

  map_publisher->publish( occupancy_grid );
}

void
Visualizer::vehicle_state_dynamic_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg )
{
  // Add the latest odometry message to the buffer with its timestamp
  latest_state = dynamics::conversions::to_cpp_type( msg );
  state_buffer.add( msg );

  // Convert the buffer to a marker array representing the driven path
  auto driven_path_array            = conversions::state_buffer_to_markers( state_buffer, offset );
  markers_to_publish["driven_path"] = driven_path_array;

  // Convert the message to MarkerArray
  auto odom_marker_array = conversions::state_to_markers( msg, offset );

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
Visualizer::borders_callback( const adore_ros2_msgs::msg::BorderArray& msg )
{
  // Convert the message to MarkerArray
  auto marker_array = conversions::border_data_to_marker_array( msg, offset );

  // Publish the MarkerArray
  markers_to_publish["borders"] = marker_array;
}

void
Visualizer::map_callback( const adore_ros2_msgs::msg::Map& msg )
{
  // Convert the message to MarkerArray
  auto marker_array = conversions::map_to_marker_array( msg, offset );

  // Publish the MarkerArray
  markers_to_publish["local_map"] = marker_array;
}

void
Visualizer::route_callback( const adore_ros2_msgs::msg::Route& msg )
{
  // Convert the message to MarkerArray
  auto marker_array = conversions::route_to_marker_array( msg, offset );

  // Publish the MarkerArray
  markers_to_publish["route"] = marker_array;
}

void
Visualizer::goal_callback( const adore_ros2_msgs::msg::GoalPoint& msg )
{
  // Convert the message to MarkerArray
  auto marker_array = conversions::goal_to_marker_array( msg, offset );

  // Publish the MarkerArray
  markers_to_publish["goal"] = marker_array;
}

void
Visualizer::traffic_participants_callback( const adore_ros2_msgs::msg::TrafficParticipantSet& msg )
{
  // Convert the message to MarkerArray
  auto marker_array = conversions::traffic_participants_to_markers( msg, offset );

  // Publish the MarkerArray
  markers_to_publish["traffic_participants"] = marker_array;
}

void
Visualizer::ignored_participants_callback( const adore_ros2_msgs::msg::TrafficParticipantSet& msg )
{
  // Convert the message to MarkerArray
  auto marker_array = conversions::ignored_participants_to_markers( msg, offset );

  // Publish the MarkerArray
  markers_to_publish["ignored_participants"] = marker_array;
}

void
Visualizer::decision_callback( const adore_ros2_msgs::msg::Trajectory& msg )
{
  MarkerArray marker_array                  = conversions::trajectory_to_markers( msg, offset );
  markers_to_publish["trajectory_decision"] = marker_array;
  trajectory_publishers["trajectory_decision"]->publish( dynamics::conversions::transpose( msg ) );
}

void
Visualizer::trajectory_callback( const adore_ros2_msgs::msg::Trajectory& msg )
{
  MarkerArray marker_array                 = conversions::trajectory_to_markers( msg, offset );
  markers_to_publish["planned_trajectory"] = marker_array;
  trajectory_publishers["planned_trajectory"]->publish( dynamics::conversions::transpose( msg ) );
}

void
Visualizer::controller_trajectory_callback( const adore_ros2_msgs::msg::Trajectory& msg )
{
  MarkerArray marker_array                    = conversions::trajectory_to_markers( msg, offset );
  markers_to_publish["controller_trajectory"] = marker_array;
  trajectory_publishers["controller_trajectory"]->publish( dynamics::conversions::transpose( msg ) );
}

void
Visualizer::safety_corridor_callback( const adore_ros2_msgs::msg::SafetyCorridor& msg )
{
  MarkerArray marker_array              = conversions::safety_corridor_to_markers( msg, offset );
  markers_to_publish["safety_corridor"] = marker_array;
}

void
Visualizer::traffic_prediction_callback( const adore_ros2_msgs::msg::TrafficPrediction& msg )
{
  MarkerArray marker_array                 = conversions::traffic_prediction_to_markers( msg, offset );
  markers_to_publish["traffic_prediction"] = marker_array;
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
