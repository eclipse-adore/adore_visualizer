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
#include "infrastructure_visualizer.hpp"
#include <rmw/rmw.h>
#include <dynamics/vehicle_state.hpp>
#include "adore_ros2_msgs/msg/infrastructure_info.hpp"
#include "color_palette.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_primitives.hpp"

using namespace std::chrono_literals;

namespace adore
{
namespace visualizer
{

InfrastructureVisualizer::InfrastructureVisualizer() : Node( "infrastructure_visualizer_node" )
{
  load_parameters();
  create_subscribers();
  create_publishers();

  visualisation_offset_transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>( this );
}

void InfrastructureVisualizer::load_parameters()
{
  declare_parameter( "asset folder", "" );
  get_parameter( "asset folder", maps_folder );

  declare_parameter( "map_image_api_key", "" );
  get_parameter( "map_image_api_key", map_image_api_key );

  declare_parameter( "map_image_grayscale", true );
  get_parameter( "map_image_grayscale", map_image_grayscale );

  declare_parameter( "visualize_infrastructure", false );
  get_parameter( "visualize_infrastructure", visualize_infrastructure);
  
  declare_parameter( "visualize_local_map", false );
  get_parameter( "visualize_local_map", visualize_local_map);

  declare_parameter( "visualize_validity_area", false );
  get_parameter( "visualize_validity_area", visualize_validity_area);

  declare_parameter( "visualize_map_image", false );
  get_parameter( "visualize_map_image", visualize_map_image);

  declare_parameter( "visualize_traffic_participants", false );
  get_parameter( "visualize_traffic_participants", visualize_traffic_participants);
}

void InfrastructureVisualizer::create_subscribers()
{
  subscriber_infrastructure_info = create_subscription<adore_ros2_msgs::msg::InfrastructureInfo>(
    "infrastructure_info", 10, std::bind( &InfrastructureVisualizer::infrastructure_info_callback, this, std::placeholders::_1 ) );

  if ( visualize_local_map )
  {
    subscriber_local_map = create_subscription<adore_ros2_msgs::msg::Map>(
      "local_map", 10, std::bind( &InfrastructureVisualizer::local_map_callback, this, std::placeholders::_1 ) );
  }

  if ( visualize_traffic_participants )
  {
    subscriber_traffic_participant_set = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>(
      "traffic_participants_with_trajectories", 10, std::bind( &InfrastructureVisualizer::traffic_participant_callback, this, std::placeholders::_1 ) );
  }

  main_timer = create_wall_timer( 100ms, std::bind( &InfrastructureVisualizer::timer_callback, this ) );
}

void InfrastructureVisualizer::create_publishers()
{
  if ( visualize_infrastructure )
  {
    publisher_infrastructure_marker = create_publisher<visualization_msgs::msg::MarkerArray>("visualize_infrastructure", 10 );
  }

  if ( visualize_local_map )
  {
    publisher_local_map_markers = create_publisher<visualization_msgs::msg::MarkerArray>("visualize_local_map", 10 );
  }

  if ( visualize_validity_area )
  {
    publisher_validation_area = create_publisher<visualization_msgs::msg::MarkerArray>("visualize_validity_area", 10 );
  }

  if ( visualize_map_image )
  {
    publisher_map_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("visualize_map_image", 10 );
  }

  if ( visualize_traffic_participants )
  {
    publisher_traffic_participant_markers = create_publisher<visualization_msgs::msg::MarkerArray>("visualize_traffic_participants", 10 );
  }
}

void InfrastructureVisualizer::timer_callback()
{
  if ( !latest_infrastructure_info.has_value() )
  {
    return;
  }
  
  std::string ns = get_namespace();
  std::string frame_id = ns + "_visualization_offset";
  publish_visualization_offset();  

  if ( visualize_infrastructure )
  {
    visualization_msgs::msg::Marker infrastructure_3d_object = primitives::create_3d_object_marker(
                                                                                                  latest_infrastructure_info.value().position_x,
                                                                                                  latest_infrastructure_info.value().position_y,
                                                                                                  0.0,
                                                                                                  1,
                                                                                                  latest_infrastructure_info.value().yaw,
                                                                                                  "infrastrucutre_visualization",
                                                                                                  0,
                                                                                                  colors::white,
                                                                                                  "low_poly_trailer_model.dae",
                                                                                                  Offset { latest_infrastructure_info.value().position_x, latest_infrastructure_info.value().position_y });
    infrastructure_3d_object.header.frame_id = ns + "_visualization_offset";
    infrastructure_3d_object.frame_locked                = true;
    infrastructure_3d_object.mesh_use_embedded_materials = true;

    MarkerArray infrastructure_makers;
    infrastructure_makers.markers.push_back(infrastructure_3d_object);
      
    publisher_infrastructure_marker->publish(infrastructure_makers);
    
  }

  if ( visualize_local_map && latest_local_map.has_value() )
  {
    visualization_msgs::msg::MarkerArray local_map_marker = conversions::to_marker_array(latest_local_map.value(), Offset { latest_infrastructure_info.value().position_x, latest_infrastructure_info.value().position_y}, frame_id);
    publisher_local_map_markers->publish(local_map_marker);
  }

  if ( visualize_validity_area )
  {
    visualization_msgs::msg::MarkerArray validity_area_marker = conversions::to_marker_array(latest_infrastructure_info.value().validity_area, Offset { latest_infrastructure_info.value().position_x, latest_infrastructure_info.value().position_y}, frame_id);
    publisher_validation_area->publish(validity_area_marker);
  }
  
  if ( visualize_map_image )
  {
    dynamics::VehicleStateDynamic state_as_vehicle;
    state_as_vehicle.x = latest_infrastructure_info.value().position_x;
    state_as_vehicle.y = latest_infrastructure_info.value().position_y;
    auto index_and_tile = map_image::generate_pointcloud2( Offset { latest_infrastructure_info.value().position_x, latest_infrastructure_info.value().position_y}, state_as_vehicle, maps_folder, false, tile_cache, map_image_api_key, frame_id );

    if( latest_tile_index != index_and_tile.first && index_and_tile.second.data.size() > 0 )
    {
      latest_tile_index = index_and_tile.first;
      publisher_map_cloud->publish( index_and_tile.second );
    }
    
  }

  if ( visualize_traffic_participants && latest_traffic_participant_set.has_value())
  {
    visualization_msgs::msg::MarkerArray traffic_participant_set_maker = conversions::to_marker_array(latest_traffic_participant_set.value(), Offset { latest_infrastructure_info.value().position_x, latest_infrastructure_info.value().position_y}, frame_id);
    publisher_traffic_participant_markers->publish(traffic_participant_set_maker);
  }
}

void InfrastructureVisualizer::publish_visualization_offset()
{
  if ( !latest_infrastructure_info.has_value() )
    return;

  geometry_msgs::msg::TransformStamped viz_transform;

  viz_transform.header.stamp    = this->get_clock()->now();
  viz_transform.header.frame_id = "world";

  std::string ns = get_namespace();
  viz_transform.child_frame_id  = ns + "_visualization_offset";

  viz_transform.transform.translation.x = latest_infrastructure_info.value().position_x;
  viz_transform.transform.translation.y = latest_infrastructure_info.value().position_y;
  viz_transform.transform.translation.z = 0;

  tf2::Quaternion q;
  q.setRPY( 0, 0, 0 );
  viz_transform.transform.rotation.x = q.x();
  viz_transform.transform.rotation.y = q.y();
  viz_transform.transform.rotation.z = q.z();
  viz_transform.transform.rotation.w = q.w();

  visualisation_offset_transform_broadcaster->sendTransform( viz_transform );
}

void InfrastructureVisualizer::infrastructure_info_callback(const adore_ros2_msgs::msg::InfrastructureInfo& msg)
{
  latest_infrastructure_info = msg;
}

void InfrastructureVisualizer::local_map_callback(const adore_ros2_msgs::msg::Map& msg)
{
  latest_local_map = msg;
}

void InfrastructureVisualizer::traffic_participant_callback(const adore_ros2_msgs::msg::TrafficParticipantSet& msg)
{
  latest_traffic_participant_set = msg;
}

} // namespace visualizer
} // namespace adore

int
main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );

  std::shared_ptr<adore::visualizer::InfrastructureVisualizer> node = std::make_shared<adore::visualizer::InfrastructureVisualizer>();
  rclcpp::spin( node );
  rclcpp::shutdown();
}
