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

#include "visualizer_conversions.hpp"

#include <adore_map/lat_long_conversions.hpp>

#include "color_palette.hpp"
#include "visualization_primitives.hpp"
#include <nlohmann/json.hpp>
#include <rclcpp/duration.hpp>

namespace adore
{
namespace visualizer
{
namespace conversions
{

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::SafetyCorridor& safety_corridor )
{
  MarkerArray marker_array;

  auto line_marker_left = primitives::create_line_marker( safety_corridor.left_border, "safety_left_border", 0, 0.6, colors::soft_red );
  marker_array.markers.push_back( line_marker_left );

  auto line_marker_right = primitives::create_line_marker( safety_corridor.right_border, "safety_right_border", 0, 0.6, colors::soft_red );
  marker_array.markers.push_back( line_marker_right );

  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::Map& local_map_msg )
{
  MarkerArray marker_array;

  for( const auto& road : local_map_msg.roads )
  {
    for( const auto& lane : road.lanes )
    {
      auto inner_marker  = primitives::create_line_marker( lane.inner_points, "inner", lane.id, 0.15, colors::white );
      auto outer_marker  = primitives::create_line_marker( lane.outer_points, "outer", lane.id, 0.15, colors::white );
      auto center_marker = primitives::create_line_marker( lane.center_points, "center", lane.id, 0.1, colors::gray );

      auto road_marker = primitives::create_lane_marker( lane.inner_points, lane.outer_points, "road", lane.id, colors::gray );

      inner_marker.lifetime  = rclcpp::Duration::from_seconds( 5.0 );
      outer_marker.lifetime  = rclcpp::Duration::from_seconds( 5.0 );
      center_marker.lifetime = rclcpp::Duration::from_seconds( 5.0 );
      road_marker.lifetime   = rclcpp::Duration::from_seconds( 5.0 );

      marker_array.markers.push_back( inner_marker );
      marker_array.markers.push_back( outer_marker );
      marker_array.markers.push_back( center_marker );
      marker_array.markers.push_back( road_marker );
    }
  }

  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::Route& route )
{

  MarkerArray marker_array;

  auto line_marker = primitives::create_line_marker( route.center_points, "route", 0, 0.2, colors::yellow );
  marker_array.markers.push_back( line_marker );

  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::GoalPoint& goal )
{
  MarkerArray marker_array;

  // Parameters for the finish line marker
  double square_size = 0.5; // Size of each square in the finish line marker
  int    grid_cols   = 4;   // Number of columns in the finish line grid

  // Create the finish line marker
  MarkerArray finish_line_markers = primitives::create_finish_line_marker( goal.x_position, goal.y_position, square_size );

  // Create the goal text marker
  double      text_size  = 2.0; // Size of the text
  std::string text       = "Goal";
  Color       text_color = colors::black;
  std::string ns         = "goal_text";

  double finish_line_width = grid_cols * square_size;
  double text_spacing      = 0.5; // Additional spacing between the finish line and the text

  double text_x_position = goal.x_position + finish_line_width + text_spacing; // Position text to the right of the finish line
  double text_y_position = goal.y_position;                                    // Align vertically with the finish line

  // Create the text markers
  Marker text_markers = primitives::create_text_marker( text_x_position, text_y_position, 0.0, text, text_size, text_color, ns );

  // Combine the markers from both arrays
  marker_array.markers.insert( marker_array.markers.end(), finish_line_markers.markers.begin(), finish_line_markers.markers.end() );
  marker_array.markers.push_back( text_markers );

  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::TrafficParticipantSet& participant_set )
{
  MarkerArray marker_array;

  for( const auto& participant : participant_set.data )
  {
    const auto& state   = participant.participant_data.motion_state;
    double      heading = state.yaw_angle;

    // Compute unit vector for heading
    double unit_vector_x = std::cos( heading );
    double unit_vector_y = std::sin( heading );

    double participant_length = participant.participant_data.physical_parameters.body_length;
    double participant_width  = participant.participant_data.physical_parameters.body_width;
    double participant_height = participant.participant_data.physical_parameters.body_height;

    auto participant_color = participant.participant_data.goal_point.x < 0.01 ? colors::red : colors::purple;


    bool controllable = participant.participant_data.v2x_station_id != 0;

    Marker object_bounding_box_marker = primitives::create_rectangle_marker( state.x, state.y,
                                                                             participant_height / 2,             // Z position for height
                                                                             participant_length,                 // Length
                                                                             participant_width,                  // Width
                                                                             participant_height,                 // Height (example)
                                                                             heading,                            // Orientation
                                                                             "traffic_participant_bounding_box", // Namespace
                                                                             participant.participant_data.tracking_id, // ID
                                                                             participant_color );

    object_bounding_box_marker.lifetime = rclcpp::Duration::from_seconds( 1.0 ); // Add lifetime
    marker_array.markers.push_back( object_bounding_box_marker );

    std::optional<Marker> object_3d_model_marker = get_participant_3d_model( participant );
    if( object_3d_model_marker.has_value() )
    {
      object_3d_model_marker.value().lifetime = rclcpp::Duration::from_seconds( 1.0 ); // Add lifetime
      marker_array.markers.push_back( object_3d_model_marker.value() );
    }

    // Add velocity line marker
    geometry_msgs::msg::Point start;
    start.x = state.x;
    start.y = state.y;

    geometry_msgs::msg::Point end;
    end.x = start.x + unit_vector_x * state.vx;
    end.y = start.y + unit_vector_y * state.vx;

    static const int VEL_I = 100; // for separate id for velocity marker

    auto velocity_marker     = primitives::create_line_marker( start, end, "participant_velocity",
                                                               participant.participant_data.tracking_id + VEL_I,
                                                               0.1, // Line width
                                                               colors::orange );
    velocity_marker.type     = velocity_marker.ARROW;
    velocity_marker.lifetime = rclcpp::Duration::from_seconds( 1.0 ); // Add lifetime
    marker_array.markers.push_back( velocity_marker );

    // Add heading line marker
    geometry_msgs::msg::Point heading_end;
    heading_end.x = start.x + unit_vector_x;
    heading_end.y = start.y + unit_vector_y;

    static const int HEADING_I = 10000; // for separate id for heading marker

    auto heading_marker           = primitives::create_line_marker( start, heading_end, "participant_heading",
                                                                    participant.participant_data.tracking_id + HEADING_I,
                                                                    0.2, // Line width
                                                                    colors::gray );
    heading_marker.lifetime       = rclcpp::Duration::from_seconds( 1.0 ); // Add lifetime
    static const int TRAJECTORY_I = 1000000;
    if( participant.participant_data.predicted_trajectory.states.size() > 0 )
    {
      if( !controllable )
        participant_color[3] = 0.1;
      // Create the line marker for the trajectory
      auto line_marker = primitives::create_flat_line_marker( participant.participant_data.predicted_trajectory.states, "decision",
                                                              participant.participant_data.tracking_id + TRAJECTORY_I, 1.8,
                                                              participant_color );


      line_marker.lifetime = rclcpp::Duration::from_seconds( 0.2 );
      marker_array.markers.push_back( line_marker );
    }
    marker_array.markers.push_back( heading_marker );

    auto route_marker               = to_marker_array( participant.participant_data.route );
    route_marker.markers.front().id = 2000000 + participant.participant_data.tracking_id; // Ensure unique ID for route marker
    marker_array.markers.insert( marker_array.markers.end(), route_marker.markers.begin(), route_marker.markers.end() );
  }
  auto closed_border = participant_set.validity_area.points;
  if( closed_border.size() > 0 )
  {
    closed_border.push_back( closed_border.front() );
    auto boundary_marker = primitives::create_line_marker( closed_border, "boundary", 999, 0.2, colors::soft_red );
    marker_array.markers.push_back( boundary_marker );
  }


  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::Trajectory& trajectory )
{
  MarkerArray marker_array;

  if( trajectory.states.size() == 0 )
  {
    return marker_array;
  }
  const auto& state      = trajectory.states[0];
  double      text_size  = 1;
  Color       text_color = colors::blue;
  std::string ns         = "trajectory_label";

  // Create the text markers
  // Marker text_marker = primitives::create_text_marker( state.x, state.y, state.z + 5, trajectory.label, text_size, text_color, ns );

  // Combine the markers from both arrays
  // marker_array.markers.push_back( text_marker );
  // Create the line marker for the trajectory
  auto line_marker = primitives::create_flat_line_marker( trajectory.states, "decision", trajectory.request_id, 1.8, colors::soft_blue );
  marker_array.markers.push_back( line_marker );

  return marker_array;
}

// Conversion function for odometry to markers
MarkerArray
to_marker_array( const adore_ros2_msgs::msg::VehicleStateDynamic& msg )
{
  MarkerArray marker_array;

  auto ego_vehicle_marker = primitives::create_3d_object_marker( 0, 0,
                                                                 0.0, // Z height
                                                                 1,   // scale
                                                                 0.0, "ego_vehicle", 0, colors::blue,
                                                                 "low_poly_ngc_model.dae" ); // Create a rectangle marker for the ego
                                                                                             // vehicle

  ego_vehicle_marker.frame_locked    = true;
  ego_vehicle_marker.header.frame_id = msg.header.frame_id;

  ego_vehicle_marker.mesh_use_embedded_materials = true;
  marker_array.markers.push_back( ego_vehicle_marker );
  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::TrafficSignals& traffic_signals )
{
  MarkerArray marker_array;

  for( size_t i = 0; i < traffic_signals.signals.size(); ++i )
  {
    const auto& signal = traffic_signals.signals[i];

    // Rectangle (signal housing)
    Marker rect_marker = primitives::create_rectangle_marker( signal.x, signal.y, 0.1, 0.7, 2.4, 0.2, 0.0, "traffic_signal", i,
                                                              colors::black );
    marker_array.markers.push_back( rect_marker );

    // Circle markers for the lights
    const double circle_radius  = 0.6;
    const double between_lights = 0.75; // Spacing between centers of lights

    // Red light
    Marker red_light = primitives::create_sphere_marker( signal.x, signal.y * between_lights, 0.12, circle_radius, "traffic_signal_red", i,
                                                         ( signal.state == adore_ros2_msgs::msg::TrafficSignal::RED ) ? colors::red
                                                                                                                      : colors::gray );
    marker_array.markers.push_back( red_light );

    // Yellow light
    Marker yellow_light = primitives::create_sphere_marker( signal.x, signal.y, 0.12, circle_radius, "traffic_signal_yellow", i,
                                                            ( signal.state == adore_ros2_msgs::msg::TrafficSignal::YELLOW )
                                                              ? colors::yellow
                                                              : colors::gray );
    marker_array.markers.push_back( yellow_light );

    // Green light
    Marker green_light = primitives::create_sphere_marker( signal.x, signal.y * between_lights, 0.12, circle_radius, "traffic_signal_green",
                                                           i,
                                                           ( signal.state == adore_ros2_msgs::msg::TrafficSignal::GREEN ) ? colors::green
                                                                                                                          : colors::gray );
    marker_array.markers.push_back( green_light );
  }

  return marker_array;
}

// Conversion function for Waypoints message (just showing the points)
MarkerArray
to_marker_array( const adore_ros2_msgs::msg::Waypoints& waypoints_msg )
{
  MarkerArray marker_array;

  // Iterate over each waypoint and create a sphere marker
  for( size_t i = 0; i < waypoints_msg.waypoints.size(); ++i )
  {
    const auto& point = waypoints_msg.waypoints[i];
    // Create a sphere marker at the waypoint location (using a default scale, e.g., 0.2)
    Marker sphere_marker = primitives::create_sphere_marker( point.x, point.y, 0.5,
                                                             0.3,         // Scale (radius)
                                                             "waypoints", // Namespace
                                                             i,           // ID
                                                             colors::purple );
    marker_array.markers.push_back( sphere_marker );
  }

  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::VisualizableObject& msg )
{
  MarkerArray marker_array;

  auto object_marker = primitives::create_3d_object_marker( msg.x, msg.y,
                                                            msg.z, // Z height
                                                            1,     // scale
                                                            msg.yaw, msg.model, 0, colors::blue,
                                                            msg.model ); // Create a rectangle marker for the ego vehicle

  object_marker.frame_locked = true;


  object_marker.mesh_use_embedded_materials = true;
  marker_array.markers.push_back( object_marker );
  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::CautionZone& caution_zone )
{
  MarkerArray marker_array;

  // Check if the polygon contains points
  if( caution_zone.polygon.points.empty() )
  {
    return marker_array;
  }

  auto polygon_points = caution_zone.polygon.points;

  // Ensure that the polygon is closed. If the first and last points are not identical, add the first point at the end.
  if( polygon_points.front().x != polygon_points.back().x || polygon_points.front().y != polygon_points.back().y )
  {
    polygon_points.push_back( polygon_points.front() );
  }

  Marker line_marker = primitives::create_flat_line_marker( polygon_points, "caution_zone", 0, 0.5, colors::soft_orange );
  marker_array.markers.push_back( line_marker );

  // Optionally, add a text marker for the label of the caution zone.
  // Compute the centroid of the polygon to position the label.
  double centroid_x = 0.0;
  double centroid_y = 0.0;
  for( const auto& pt : polygon_points )
  {
    centroid_x += pt.x;
    centroid_y += pt.y;
  }
  size_t num_points = polygon_points.size();
  centroid_x        /= num_points;
  centroid_y        /= num_points;

  Marker text_markers = primitives::create_text_marker( centroid_x, centroid_y, 0.0, caution_zone.label,
                                                        1.0,            // Text size
                                                        colors::orange, // Text color
                                                        caution_zone.label );
  marker_array.markers.push_back( text_markers );

  return marker_array;
}

NavSatFix
to_nav_sat_fix( const adore_ros2_msgs::msg::VehicleStateDynamic& vehicle_state_dynamic )
{
  NavSatFix nav_sat_fix;

  std::vector<double> lat_lon = map::convert_utm_to_lat_lon( vehicle_state_dynamic.x, vehicle_state_dynamic.y, 32, "U" );

  nav_sat_fix.latitude  = lat_lon[0];
  nav_sat_fix.longitude = lat_lon[1];

  return nav_sat_fix;
}

using json = nlohmann::json;

GeoJSON
to_geo_json( const adore_ros2_msgs::msg::GoalPoint& goal_point )
{
  auto goal_position_lat_lon = map::convert_utm_to_lat_lon( goal_point.x_position, goal_point.y_position, 32, "U" );

  json goal_geojson = {
    {    "type",                                                                                                                      "FeatureCollection"},
    {"features", json::array( { { { "type", "Feature" },
 { "properties", { { "name", "My Point" }, { "style", { { "color", "#ff0000" } } } } },
 { "geometry",
 { { "type", "Point" },
 { "coordinates", json::array( { goal_position_lat_lon[1], goal_position_lat_lon[0] } ) } } } } } )                                      }
  };

  foxglove_msgs::msg::GeoJSON goal_map;
  goal_map.geojson = goal_geojson.dump();

  return goal_map;
}

GeoJSON
to_geo_json( const adore_ros2_msgs::msg::Route& route )
{
  auto route_json_array = json::array( {} );

  for( const auto& point : route.center_points )
  {
    auto route_point_lat_lon = map::convert_utm_to_lat_lon( point.x, point.y, 32, "U" );
    route_json_array.push_back( { route_point_lat_lon[1], route_point_lat_lon[0] } );
  }

  json path = {
    {    "type",                                                "FeatureCollection"         },
    {"features",
     json::array(
     { { { "type", "Feature" },
     { "properties", { { "name", "Route Path" }, { "style", { { "color", "#dfd331" }, { "weight", 3 }, { "opacity", 1.0 } } } } },
     { "geometry", { { "type", "LineString" }, { "coordinates", route_json_array } } } } } )}
  };

  foxglove_msgs::msg::GeoJSON route_map;
  route_map.geojson = path.dump();

  return route_map;
}

std::optional<Marker>
get_participant_3d_model( const adore_ros2_msgs::msg::TrafficParticipantDetection& participant )
{
  bool controllable = participant.participant_data.v2x_station_id != 0;

  const auto& state   = participant.participant_data.motion_state;
  double      heading = state.yaw_angle;

  auto participant_color = participant.participant_data.goal_point.x < 0.01 ? colors::red : colors::purple;

  if( controllable )
  {
    Marker object_3d_model_marker = primitives::create_3d_object_marker( state.x, state.y,
                                                                         0.01, // Z height
                                                                         1,    // scale
                                                                         heading, "traffc_participant_model",
                                                                         participant.participant_data.tracking_id, colors::blue,
                                                                         "low_poly_ngc_model.dae" ); // Create a rectangle marker for the
                                                                                                     // ego vehicle
    object_3d_model_marker.frame_locked                = true;
    object_3d_model_marker.mesh_use_embedded_materials = true;

    return object_3d_model_marker;
  }

  std::optional<std::string> model_name;

  auto participant_classification = participant.participant_data.classification.type_id;
  switch( participant_classification )
  {
    case adore_ros2_msgs::msg::TrafficClassification::CAR:
    {
      model_name = get_best_fiting_car_3d_model( participant );
      break;
    }
    case adore_ros2_msgs::msg::TrafficClassification::PEDESTRIAN:
    {
      model_name = "human_standing.dae";
      break;
    }
    case adore_ros2_msgs::msg::TrafficClassification::TRUCK:
    {
      model_name = "truck.dae";
      break;
    }
    default:
    {
      model_name = {};
      break;
    }
  }

  if( model_name.has_value() )
  {
    Marker object_3d_model_marker                      = primitives::create_3d_object_marker( state.x, state.y,
                                                                                              0.01, // Z height
                                                                                              1.0, heading, "traffc_participant_model",
                                                                                              participant.participant_data.tracking_id, colors::blue,
                                                                                              model_name.value() );
    object_3d_model_marker.frame_locked                = true;
    object_3d_model_marker.mesh_use_embedded_materials = true;
    return object_3d_model_marker;
  }

  return {};
}

std::string
get_best_fiting_car_3d_model( const adore_ros2_msgs::msg::TrafficParticipantDetection& participant )
{
  double participant_length = participant.participant_data.physical_parameters.body_length;

  if( participant_length > 5.0 )
  {
    return "car_bus.dae";
  }

  if( participant_length > 4.5 ) // large car (around the size of the standard vehicle)
  {
    return "car_large.dae";
  }

  if( participant_length > 4 ) // medium car
  {
    return "car_medium.dae";
  }

  return "car_small.dae";
}

} // namespace conversions
} // namespace visualizer
} // namespace adore
