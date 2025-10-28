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

  building_cache = buildings::extract_buildings_from_road_features(assets_folder);
}

void
Visualizer::create_publishers()
{
  tf_broadcaster                   = std::make_shared<tf2_ros::TransformBroadcaster>( this );
  map_grid_publisher               = create_publisher<nav_msgs::msg::OccupancyGrid>( "map_grid", 1 );
  buildings_publisher = create_publisher<visualization_msgs::msg::MarkerArray>( "buildings", 1 );
  traffic_light_behavior_publisher = create_publisher<sensor_msgs::msg::Image>( "traffic_light_behavior", 1 );
  vehicle_behavior_publisher = create_publisher<sensor_msgs::msg::Image>( "vehicle_behavior", 1 );
  map_location_publisher = create_publisher<sensor_msgs::msg::NavSatFix>( "vehicle_location", 1 );
  route_visualization_publisher = create_publisher<foxglove_msgs::msg::GeoJSON>( "route_visualization", 1 );
  goal_visualization_publisher = create_publisher<foxglove_msgs::msg::GeoJSON>( "goal_visualization", 1 );
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

  route_subscriber = create_subscription<adore_ros2_msgs::msg::Route>( "route", 1, std::bind( &Visualizer::route_callback, this, std::placeholders::_1 ) );
  decision_maker_status_subscriber = create_subscription<adore_ros2_msgs::msg::NodeStatus>( "decision_maker_status", 1, std::bind( &Visualizer::decision_maker_status_callback, this, std::placeholders::_1 ) );

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
  if( !visualization_offset_center )
    return;
  publish_visualization_frame();
  publish_map_location();
}

void
Visualizer::low_frequency_timer_callback()
{
  update_all_dynamic_subscriptions();
  if( !visualization_offset_center )
    return;
  publish_map_image();
  publish_buildings();
  publish_map_route();
  publish_traffic_light_behavior();
  publish_vehicle_behavior();

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
    offset_tf.transform.translation.x = visualization_offset_center->x;
    offset_tf.transform.translation.y = visualization_offset_center->y;
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
    auto latest_state           = dynamics::conversions::to_cpp_type( msg );
    visualization_offset_center = { latest_state.x, latest_state.y };
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
    visualization_offset_center = { msg.position_x, msg.position_y };
  }
}

void
Visualizer::publish_map_image()
{
  if( !visualization_offset_center )
    return;

  return;
  // Generate or retrieve cached PointCloud2
  auto index_and_tile = map_image::generate_occupancy_grid( visualization_offset_center->x, visualization_offset_center->y, assets_folder + "/maps/",
                                                            false, grid_tile_cache, map_image_api_key );

  if( latest_tile_idx != index_and_tile.first && map_grid_publisher->get_subscription_count() > 0 && index_and_tile.second.data.size() > 0 )
  {
    latest_tile_idx = index_and_tile.first;
    map_grid_publisher->publish( index_and_tile.second );
  }
}

void
Visualizer::publish_buildings()
{
  MarkerArray buildings_markers;
  
  std::vector<int> buildings_to_visualize_indexes = buildings::get_nearby_buildings(building_cache, visualization_offset_center.value().x, visualization_offset_center.value().y, 200);

  for ( const int building_index : buildings_to_visualize_indexes )
  {
    const auto& building = building_cache[building_index];

    double standard_building_height = 10.0;

    Marker building_marker = primitives::create_rectangle_marker(building.position_utm_x, building.position_utm_y, standard_building_height / 2.0, building.length / 2.0, building.width / 2.0, standard_building_height, building.orientation, "building", building_index, colors::white);
    building_marker.lifetime.sec = 20.0;
    building_marker.header.frame_id = "world";
    buildings_markers.markers.push_back(building_marker);
  }

  buildings_publisher->publish( buildings_markers );
}

void
Visualizer::publish_map_location()
{
  if ( !visualization_offset_center )
    return;
  
  sensor_msgs::msg::NavSatFix nav_sat_fix;

  std::vector<double> lat_lon = map::convert_utm_to_lat_lon(visualization_offset_center.value().x, visualization_offset_center.value().y, 32, "U");

  nav_sat_fix.latitude = lat_lon[0];
  nav_sat_fix.longitude = lat_lon[1];

  map_location_publisher->publish(nav_sat_fix);
}

void
Visualizer::publish_map_route()
{
  if ( !visualization_offset_center)
    return;

  if ( !ego_vehicle_route )
    return;

  math::Point2d goal_point = { ego_vehicle_route.value().goal.x, ego_vehicle_route.value().goal.y };

  auto goal_position_lat_lon = map::convert_utm_to_lat_lon(goal_point.x, goal_point.y, 32, "U");

  json goal_geojson = {
      {"type", "FeatureCollection"},
      {"features", json::array({
        {
          {"type", "Feature"},
          {"properties", {{"name", "My Point"}, {"style", {{"color", "#ff0000"}}}}},
          {"geometry", {
            {"type", "Point"},
            {"coordinates", json::array({goal_position_lat_lon[1], goal_position_lat_lon[0]})}
          }}
        }
      })}
    };

  auto route_json_array = json::array({});

  for ( const auto& point : ego_vehicle_route.value().center_points )
  {
    auto route_point_lat_lon = map::convert_utm_to_lat_lon(point.x, point.y, 32, "U");
    route_json_array.push_back( { route_point_lat_lon[1], route_point_lat_lon[0] } );
  }

  json path = {
      {"type", "FeatureCollection"},
      {"features", json::array({
        {
          {"type", "Feature"},
          {"properties", {
            {"name", "Route Path"},
            {"style", {
              {"color", "#dfd331"},
              {"weight", 3},
              {"opacity", 1.0}
            }}
          }},
          {"geometry", {
            {"type", "LineString"},
            {"coordinates", route_json_array }
          }}
        }
      })}
    };

  foxglove_msgs::msg::GeoJSON route_map;
  route_map.geojson = path.dump();

  foxglove_msgs::msg::GeoJSON goal_map;
  goal_map.geojson = goal_geojson.dump();

  route_visualization_publisher->publish(route_map);
  goal_visualization_publisher->publish(goal_map);
}

void
Visualizer::publish_traffic_light_behavior()
{
  if( !ego_vehicle_decision_maker_status.has_value() )
    return;

  auto traffic_light_state = ego_vehicle_decision_maker_status->get_info<int>("nearest_traffic_light_signal");

  int traffic_light = 3; // default unknown state

  if ( traffic_light_state.has_value() )
    traffic_light = traffic_light_state.value();

  // @TODO, make this only trigger if there is a change!! Use an enum
  const auto& behavior_image = behavior::fetch_traffic_light_state_image(assets_folder, traffic_light); 

  if ( !behavior_image.has_value() )
  {
    std::cerr << "Tried to load behavior image, but it was not present" << std::endl;
    return;
  }
  
  sensor_msgs::msg::Image image_msg = behavior::image_to_msg( behavior_image.value() );

  traffic_light_behavior_publisher->publish(image_msg);
}

void
Visualizer::publish_vehicle_behavior()
{
  if ( !ego_vehicle_decision_maker_status.has_value() )
    return;
  
  auto distance_to_vehicle_in_front = ego_vehicle_decision_maker_status->get_info<float>("obstacle_distance");

  if ( !distance_to_vehicle_in_front.has_value() )
    return;

  std::cerr << "Distance: " << distance_to_vehicle_in_front.value() << std::endl;

  // @TODO, make this only trigger if there is a change!! Use an enum
  const auto& behavior_image = behavior::fetch_vehicle_state_image(assets_folder, distance_to_vehicle_in_front.value() ); 

  if ( !behavior_image.has_value() )
  {
    std::cerr << "Tried to load behavior image, but it was not present" << std::endl;
    return;
  }
  
  sensor_msgs::msg::Image image_msg = behavior::image_to_msg( behavior_image.value() );

  vehicle_behavior_publisher->publish(image_msg);
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

void
Visualizer::route_callback( const adore_ros2_msgs::msg::Route& msg )
{
  ego_vehicle_route = msg;
}


void
Visualizer::decision_maker_status_callback( const adore_ros2_msgs::msg::NodeStatus& msg )
{
  ego_vehicle_decision_maker_status = status::NodeStatus( msg );
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
