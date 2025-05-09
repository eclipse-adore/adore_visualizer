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
#include "vehicle_visualizer.hpp"
#include <adore_math/point.h>
#include <adore_dynamics_conversions.hpp>
#include <dynamics/traffic_participant.hpp>
#include <dynamics/trajectory.hpp>
#include <dynamics/vehicle_state.hpp>
#include "adore_ros2_msgs/msg/caution_zone.hpp"
#include "adore_ros2_msgs/msg/goal_point.hpp"
#include "adore_ros2_msgs/msg/route.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "adore_ros2_msgs/msg/trajectory.hpp"
#include "adore_ros2_msgs/msg/vehicle_state_dynamic.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_primitives.hpp"
#include "visualizer_conversions.hpp"

using namespace std::chrono_literals;

namespace adore
{
namespace visualizer
{

VehicleVisualizer::VehicleVisualizer() : Node( "vehicle_visualizer_node" )
{
  declare_parameter( "asset folder", "" );
  get_parameter( "asset folder", maps_folder );

  declare_parameter( "map_image_api_key", "" );
  get_parameter( "map_image_api_key", map_image_api_key );

  declare_parameter( "map_image_grayscale", true );
  get_parameter( "map_image_grayscale", map_image_grayscale );
  
  declare_parameter( "visualize_vehicle", false );
  get_parameter( "visualize_vehicle", visualize_vehicle);

  declare_parameter( "visualize_local_map", false );
  get_parameter( "visualize_local_map", visualize_local_map);

  declare_parameter( "visualize_route", false );
  get_parameter( "visualize_route", visualize_route);

  declare_parameter( "visualize_trajectory", false );
  get_parameter( "visualize_trajectory", visualize_trajectory);

  declare_parameter( "visualize_state", false );
  get_parameter( "visualize_state", visualize_state);
  
  declare_parameter( "visualize_goal_point", false );
  get_parameter( "visualize_goal_point", visualize_goal_point );

  declare_parameter( "visualize_map_image", false );
  get_parameter( "visualize_map_image", visualize_map_image );

  declare_parameter( "visualize_traffic_participants", false );
  get_parameter( "visualize_traffic_participants", visualize_traffic_participants);

  declare_parameter( "visualize_traffic_participants_predicted_trajectories", false );
  get_parameter( "visualize_traffic_participants_predicted_trajectories", visualize_traffic_participants_predicted_trajectories);

  declare_parameter( "visualize_ignored_traffic_participants", false );
  get_parameter( "visualize_ignored_traffic_participants", visualize_ignored_traffic_participants);

  declare_parameter( "visualize_remote_operations ", false );
  get_parameter( "visualize_remote_operations ", visualize_remote_operations );

  declare_parameter( "visualize_infrastructure_validity_area", false );
  get_parameter( "visualize_infrastructure_validity_area", visualize_infrastructure_validity_area);

  create_subscribers();
  create_publishers();
}

void VehicleVisualizer::create_subscribers()
{
  vehicle_transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>( *this );
  visualisation_offset_transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>( this );
  
  subscriber_vehicle_state_dynamic = create_subscription<adore_ros2_msgs::msg::VehicleStateDynamic>(
    "vehicle_state/dynamic", 10, std::bind( &VehicleVisualizer::vehicle_state_dynamic_callback, this, std::placeholders::_1 ) );

  if ( visualize_trajectory || visualize_state )
  {
    subscriber_trajectory = create_subscription<adore_ros2_msgs::msg::Trajectory>(
      "trajectory_decision", 10, std::bind( &VehicleVisualizer::planned_trajectory_callback, this, std::placeholders::_1 ) );
  }

  if ( visualize_local_map )
  {
    subscriber_local_map = create_subscription<adore_ros2_msgs::msg::Map>(
      "local_map", 10, std::bind( &VehicleVisualizer::local_map_callback, this, std::placeholders::_1 ) );
  }

  if ( visualize_route )
  {
    subscriber_route = create_subscription<adore_ros2_msgs::msg::Route>(
      "route", 10, std::bind( &VehicleVisualizer::route_callback, this, std::placeholders::_1 ) );
  }

  if ( visualize_goal_point )
  {
    subscriber_goal_point = create_subscription<adore_ros2_msgs::msg::GoalPoint>(
      "mission/goal_position", 10, std::bind( &VehicleVisualizer::goal_point_callback, this, std::placeholders::_1 ) );
  }

  if ( visualize_traffic_participants )
  {
    subscriber_traffic_participants = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>(
      "traffic_participants", 10, std::bind( &VehicleVisualizer::traffic_participant_set_callback, this, std::placeholders::_1 ) );
  }
  if ( visualize_traffic_participants_predicted_trajectories )
  {
    subscriber_traffic_participants_predicted_trajectories = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>(
      "traffic_prediction", 10, std::bind( &VehicleVisualizer::traffic_participant_set_with_predictions_callback, this, std::placeholders::_1 ) );
  }

  if ( visualize_ignored_traffic_participants )
  {
    subscriber_ignored_traffic_participants = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>(
      "ignored_participants", 10, std::bind( &VehicleVisualizer::ignored_traffic_participant_set_callback, this, std::placeholders::_1 ) );
  }

  if ( visualize_remote_operations )
  {
    subscriber_caution_zone = create_subscription<adore_ros2_msgs::msg::CautionZone>(
      "caution_zones", 10, std::bind( &VehicleVisualizer::caution_zone_callback, this, std::placeholders::_1 ) );
  }

  if ( visualize_infrastructure_validity_area )
  {
    subscriber_infrastructure_traffic_participants = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>(
      "infrastructure_traffic_participants", 10, std::bind( &VehicleVisualizer::infrastructure_traffic_participants, this, std::placeholders::_1 ) );
    
  }

  main_timer = create_wall_timer( 100ms, std::bind( &VehicleVisualizer::timer_callback, this ) );  
}

void VehicleVisualizer::create_publishers()
{
  if ( visualize_vehicle )
  {
    publisher_vehicle_markers = create_publisher<visualization_msgs::msg::MarkerArray>("visualize_vehicle", 10 );
  }

  if ( visualize_trajectory )
  {
    publisher_trajectory_markers = create_publisher<visualization_msgs::msg::MarkerArray>("visualize_trajectory", 10 );
  }

  if ( visualize_state )
  {
    publisher_state_markers = create_publisher<visualization_msgs::msg::MarkerArray>("visualize_state", 10 );
  }

  if ( visualize_local_map )
  {
    publisher_local_map_markers = create_publisher<visualization_msgs::msg::MarkerArray>("visualize_local_map", 10 );
  }

  if ( visualize_route )
  {
    publisher_route_markers = create_publisher<visualization_msgs::msg::MarkerArray>("visualize_route", 10 );
  }

  if ( visualize_remote_operations )
  {
    publisher_caution_zones = create_publisher<visualization_msgs::msg::MarkerArray>("visualize_caution_zones", 10 );
  }

  if ( visualize_goal_point )
  {
    publisher_goal_point_markers = create_publisher<visualization_msgs::msg::MarkerArray>("visualize_goal_point", 10 );
  }

  if ( visualize_traffic_participants )
  {
    publisher_traffic_participant_markers = create_publisher<visualization_msgs::msg::MarkerArray>("visualize_traffic_participants", 10 );
  }

  if ( visualize_ignored_traffic_participants )
  {
    publisher_ignored_traffic_participant_markers = create_publisher<visualization_msgs::msg::MarkerArray>("visualize_ignored_traffic_participants", 10 );
  }

  if (visualize_traffic_participants_predicted_trajectories )
  {
    publisher_traffic_participant_predicted_trajectories = create_publisher<visualization_msgs::msg::MarkerArray>("visualize_traffic_participant_predicted_trajectories", 10 );
  }

  if (visualize_infrastructure_validity_area )
  {
    publisher_infrastructure_area = create_publisher<visualization_msgs::msg::MarkerArray>("visualize_infrastructure_traffic_participants", 10 );
  }

  if ( visualize_map_image )
  {
    publisher_map_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("visualize_map_image", 10 );
  }
}

void VehicleVisualizer::timer_callback()
{
  current_time = now();
  if ( !latest_vehicle_state_dynamic.has_value() || !visualization_offset.has_value() )
    return;

  publish_visualization_offset();  

  std::string ns_prefix = get_namespace();
  std::string frame_id = ns_prefix + "_visualization_offset";

  dynamics::VehicleStateDynamic state = dynamics::conversions::to_cpp_type(latest_vehicle_state_dynamic.value());

  if ( visualize_vehicle )
  {
    visualization_msgs::msg::MarkerArray vehicle_marker = conversions::to_marker_array(latest_vehicle_state_dynamic.value(), visualization_offset.value(), ns_prefix);
    publisher_vehicle_markers->publish(vehicle_marker);
  }

  if ( visualize_trajectory && latest_trajectory.has_value() )
  {
    visualization_msgs::msg::MarkerArray planned_trajectory_marker = conversions::to_marker_array(latest_trajectory.value(), visualization_offset.value(), frame_id);
    publisher_trajectory_markers->publish(planned_trajectory_marker);
  }

  if ( visualize_state && latest_trajectory.has_value() )
  {
    visualization_msgs::msg::MarkerArray state_marker = conversions::to_marker_array(latest_trajectory.value(), visualization_offset.value(), frame_id, 0.5);
    publisher_state_markers ->publish( state_marker );
    
  }

  
  if ( visualize_local_map && latest_local_map.has_value() )
  {
    visualization_msgs::msg::MarkerArray local_map_marker = conversions::to_marker_array(latest_local_map.value(), visualization_offset.value(), frame_id);
    publisher_local_map_markers->publish(local_map_marker);
  }

  if ( visualize_goal_point && latest_goal_point.has_value() )
  {
    visualization_msgs::msg::MarkerArray goal_point_marker = conversions::to_marker_array(latest_goal_point.value(), visualization_offset.value(), frame_id);
    publisher_goal_point_markers->publish(goal_point_marker);
  }

  if ( visualize_remote_operations )
  {
    if ( latest_caution_zone.has_value() )
    {
      visualization_msgs::msg::MarkerArray causion_zone_marker = conversions::to_marker_array(latest_caution_zone.value(), visualization_offset.value(), frame_id);
      publisher_caution_zones->publish(causion_zone_marker );
    }
  }

  if ( visualize_route && latest_route.has_value() )
  {
    visualization_msgs::msg::MarkerArray route_marker = conversions::to_marker_array(latest_route.value(), visualization_offset.value(), frame_id);
    publisher_route_markers->publish(route_marker);
  }
    // auto vehicle_marker = conversions::

  if ( visualize_traffic_participants && latest_traffic_participant_set.has_value() )
  {
    visualization_msgs::msg::MarkerArray traffic_participant_set_marker = conversions::to_marker_array(latest_traffic_participant_set.value(), visualization_offset.value(), frame_id);
    publisher_traffic_participant_markers->publish(traffic_participant_set_marker);
  }

  if ( visualize_ignored_traffic_participants && latest_ignored_traffic_participant_set.has_value() )
  {
    visualization_msgs::msg::MarkerArray traffic_participant_set_marker = conversions::to_marker_array(latest_ignored_traffic_participant_set.value(), visualization_offset.value(), frame_id);
    publisher_ignored_traffic_participant_markers->publish(traffic_participant_set_marker);
  }

  if ( visualize_traffic_participants_predicted_trajectories && latest_traffic_participants_set_with_predictions.has_value() )
  {
    visualization_msgs::msg::MarkerArray traffic_participant_predicted_trajectories_markers;
    
    dynamics::TrafficParticipantSet tps = dynamics::conversions::to_cpp_type( latest_traffic_participants_set_with_predictions.value() );
    
    for (const auto& [id, participant] : tps.participants )
    {
      if ( !participant.trajectory.has_value() )
        continue;

      adore_ros2_msgs::msg::Trajectory participant_trajectory = dynamics::conversions::to_ros_msg( participant.trajectory.value() );
      visualization_msgs::msg::MarkerArray trajectory_markers = conversions::to_marker_array( participant_trajectory, visualization_offset.value(), frame_id);

      for ( const auto& trajectory_points : trajectory_markers.markers )
      {
        traffic_participant_predicted_trajectories_markers.markers.push_back(trajectory_points);
      }
    }
    publisher_traffic_participant_predicted_trajectories->publish( traffic_participant_predicted_trajectories_markers );
  }
    
  if ( visualize_map_image )
  {
    auto index_and_tile = map_image::generate_pointcloud2( visualization_offset.value(), state, maps_folder, false, tile_cache, map_image_api_key, frame_id );

    if( latest_tile_index != index_and_tile.first && index_and_tile.second.data.size() > 0 )
    {
      latest_tile_index = index_and_tile.first;
      publisher_map_cloud->publish( index_and_tile.second );
    }
  }
  
  if ( visualize_infrastructure_validity_area && latest_infrastructure_traffic_participants.has_value() )
  {
    auto validity_area = latest_infrastructure_traffic_participants.value().validity_area;
    auto validty_area_markers = conversions::to_marker_array( validity_area, visualization_offset.value(), frame_id );
    publisher_infrastructure_area->publish( validty_area_markers );
  }

  
  last_update_time = current_time;
}

void VehicleVisualizer::vehicle_state_dynamic_callback(const adore_ros2_msgs::msg::VehicleStateDynamic& msg)
{
  if ( !latest_vehicle_state_dynamic.has_value() ) // this will only get triggered once
  {
    visualization_offset = Offset {msg.x, msg.y};
  }

  latest_vehicle_state_dynamic = msg;

}

void VehicleVisualizer::local_map_callback(const adore_ros2_msgs::msg::Map& msg)
{
  latest_local_map = msg;
}


void VehicleVisualizer::publish_visualization_offset()
{
  if ( !visualization_offset.has_value() )
    return;
  
  dynamics::VehicleStateDynamic state = dynamics::conversions::to_cpp_type(latest_vehicle_state_dynamic.value());
  state.z = 0;
  auto vehicle_frame = dynamics::conversions::vehicle_state_to_transform( state , last_update_time, get_namespace() );
  vehicle_transform_broadcaster->sendTransform( vehicle_frame );
  
  geometry_msgs::msg::TransformStamped viz_transform;

  viz_transform.header.stamp    = this->get_clock()->now();
  viz_transform.header.frame_id = "world";

  std::string ns = get_namespace();
  viz_transform.child_frame_id  = ns + "_visualization_offset";

  viz_transform.transform.translation.x = visualization_offset.value().x;
  viz_transform.transform.translation.y = visualization_offset.value().y;
  viz_transform.transform.translation.z = 0;

  tf2::Quaternion q;
  q.setRPY( 0, 0, 0 );
  viz_transform.transform.rotation.x = q.x();
  viz_transform.transform.rotation.y = q.y();
  viz_transform.transform.rotation.z = q.z();
  viz_transform.transform.rotation.w = q.w();

  visualisation_offset_transform_broadcaster->sendTransform( viz_transform );
}

void VehicleVisualizer::planned_trajectory_callback(const adore_ros2_msgs::msg::Trajectory& msg)
{
  latest_trajectory = msg;
}

void VehicleVisualizer::goal_point_callback(const adore_ros2_msgs::msg::GoalPoint& msg)
{
  latest_goal_point = msg;
}

void VehicleVisualizer::route_callback(const adore_ros2_msgs::msg::Route& msg)
{
  latest_route = msg;
}

void VehicleVisualizer::traffic_participant_set_callback(const adore_ros2_msgs::msg::TrafficParticipantSet& msg)
{
  latest_traffic_participant_set = msg;
}

void VehicleVisualizer::traffic_participant_set_with_predictions_callback(const adore_ros2_msgs::msg::TrafficParticipantSet& msg)
{
  latest_traffic_participants_set_with_predictions = msg;
}

void VehicleVisualizer::ignored_traffic_participant_set_callback(const adore_ros2_msgs::msg::TrafficParticipantSet& msg)
{
  latest_ignored_traffic_participant_set = msg;
}

void VehicleVisualizer::caution_zone_callback(const adore_ros2_msgs::msg::CautionZone& msg)
{
  latest_caution_zone = msg;
}

void VehicleVisualizer::infrastructure_traffic_participants(const adore_ros2_msgs::msg::TrafficParticipantSet& msg)
{
  latest_infrastructure_traffic_participants = msg;  
}

} // namespace visualizer
} // namespace adore

int
main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );

  std::shared_ptr<adore::visualizer::VehicleVisualizer> node = std::make_shared<adore::visualizer::VehicleVisualizer>();
  rclcpp::spin( node );
  rclcpp::shutdown();
}
