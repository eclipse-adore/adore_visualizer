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
#include "visualizer_conversions.hpp"

#include "color_palette.hpp"
#include "visualization_primitives.hpp"

namespace adore
{
namespace visualizer
{
namespace conversions
{
// Conversion function for time-based odometry buffer to a marker array using the templated line drawing primitive
MarkerArray
to_marker_array( const StateBuffer& state_buffer, const Offset& offset )
{
  MarkerArray marker_array;
  const auto& buffer = state_buffer.get();

  if( buffer.size() < 2 )
  {
    return marker_array; // Not enough points to create a line
  }

  Marker line_marker = primitives::create_flat_line_marker( buffer, "driven_path", 1, 1.8, colors::soft_gray, offset );


  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::SafetyCorridor& safety_corridor, const Offset& offset )
{
  MarkerArray marker_array;

  auto line_marker_left = primitives::create_line_marker( safety_corridor.left_border, "safety_left_border", 0, 0.6, colors::soft_red,
                                                          offset );
  marker_array.markers.push_back( line_marker_left );

  auto line_marker_right = primitives::create_line_marker( safety_corridor.right_border, "safety_right_border", 0, 0.6, colors::soft_red,
                                                           offset );
  marker_array.markers.push_back( line_marker_right );

  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::Map& local_map_msg, const Offset& offset )
{
  MarkerArray marker_array;

  for( const auto& road : local_map_msg.roads )
  {
    for( const auto& lane : road.lanes )
    {
      auto inner_marker  = primitives::create_line_marker( lane.inner_points, "inner", lane.id, 0.15, colors::white, offset );
      auto outer_marker  = primitives::create_line_marker( lane.outer_points, "outer", lane.id, 0.15, colors::white, offset );
      auto center_marker = primitives::create_line_marker( lane.center_points, "center", lane.id, 0.1, colors::gray, offset );
      marker_array.markers.push_back( inner_marker );
      marker_array.markers.push_back( outer_marker );
      marker_array.markers.push_back( center_marker );
    }
  }


  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::Route& route, const Offset& offset )
{

  MarkerArray marker_array;

  auto line_marker = primitives::create_line_marker( route.center_points, "route", 0, 0.2, colors::yellow, offset );
  marker_array.markers.push_back( line_marker );

  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::GoalPoint& goal, const Offset& offset )
{
  MarkerArray marker_array;

  // Parameters for the finish line marker
  double square_size = 0.5; // Size of each square in the finish line marker
  int    grid_cols   = 4;   // Number of columns in the finish line grid

  // Create the finish line marker
  MarkerArray finish_line_markers = primitives::create_finish_line_marker( goal.x_position, goal.y_position, square_size, offset );

  // Create the goal text marker
  double      text_size  = 1.0; // Size of the text
  std::string text       = "Goal is here";
  Color       text_color = colors::cyan;
  std::string ns         = "goal_text";

  double finish_line_width = grid_cols * square_size;
  double text_spacing      = 0.5; // Additional spacing between the finish line and the text

  double text_x_position = goal.x_position + finish_line_width + text_spacing; // Position text to the right of the finish line
  double text_y_position = goal.y_position;                                    // Align vertically with the finish line

  // Create the text markers
  MarkerArray text_markers = primitives::create_text_marker( text_x_position, text_y_position, text, text_size, text_color, ns, offset );

  // Combine the markers from both arrays
  marker_array.markers.insert( marker_array.markers.end(), finish_line_markers.markers.begin(), finish_line_markers.markers.end() );
  marker_array.markers.insert( marker_array.markers.end(), text_markers.markers.begin(), text_markers.markers.end() );

  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::TrafficParticipantSet& participant_set, const Offset& offset )
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

    auto participant_color = participant.participant_data.v2x_station_id > 0 ? colors::soft_blue : colors::soft_purple;

    Marker object_marker;

    if ( participant.participant_data.v2x_station_id != 0 )
    {
      object_marker = primitives::create_3d_object_marker( state.x, state.y,
                                                                0.01, // Z height
                                                                1,     // scale
                                                                heading,
                                                                "traffc_participant",
                                                                participant.participant_data.tracking_id,
                                                                colors::blue,
                                                                "low_poly_ngc_model.dae",
                                                                offset ); // Create a rectangle marker for the ego vehicle
      object_marker.frame_locked    = true;
      object_marker.header.frame_id = "visualization_offset";

      object_marker.mesh_use_embedded_materials = true;
      
    }
    else
    {
      
        object_marker     = primitives::create_rectangle_marker( state.x, state.y,
                                                                         1,                                     // Z position for height
                                                                         participant_length,                       // Length
                                                                         participant_width,                        // Width
                                                                         participant_height,                       // Height (example)
                                                                         heading,                                  // Orientation
                                                                         "traffic_participant",                    // Namespace
                                                                         participant.participant_data.tracking_id, // ID
                                                                         participant_color, offset );
    }

    object_marker.lifetime = rclcpp::Duration::from_seconds( 0.1 ); // Add lifetime
    marker_array.markers.push_back( object_marker );

    // Add velocity line marker
    geometry_msgs::msg::Point start;
    start.x = state.x;
    start.y = state.y;

    geometry_msgs::msg::Point end;
    end.x = start.x + unit_vector_x * state.vx;
    end.y = start.y + unit_vector_y * state.vx;

    static const int VEL_ID_OFFSET = 100; // for separate id for velocity marker

    auto velocity_marker     = primitives::create_line_marker( start, end, "participant_velocity",
                                                               participant.participant_data.tracking_id + VEL_ID_OFFSET,
                                                               0.1, // Line width
                                                               colors::orange, offset );
    velocity_marker.type     = velocity_marker.ARROW;
    velocity_marker.lifetime = rclcpp::Duration::from_seconds( 0.1 ); // Add lifetime
    marker_array.markers.push_back( velocity_marker );

    // Add heading line marker
    geometry_msgs::msg::Point heading_end;
    heading_end.x = start.x + unit_vector_x;
    heading_end.y = start.y + unit_vector_y;

    static const int HEADING_ID_OFFSET = 10000; // for separate id for heading marker

    auto heading_marker                   = primitives::create_line_marker( start, heading_end, "participant_heading",
                                                                            participant.participant_data.tracking_id + HEADING_ID_OFFSET,
                                                                            0.2, // Line width
                                                                            colors::gray, offset );
    heading_marker.lifetime               = rclcpp::Duration::from_seconds( 0.1 ); // Add lifetime
    static const int TRAJECTORY_ID_OFFSET = 1000000;
    if( participant.participant_data.predicted_trajectory.states.size() > 0 )
    {
      // Create the line marker for the trajectory
      auto line_marker = primitives::create_flat_line_marker( participant.participant_data.predicted_trajectory.states, "decision",
                                                              participant.participant_data.tracking_id + TRAJECTORY_ID_OFFSET, 1.8,
                                                              participant_color, offset );


      line_marker.lifetime = rclcpp::Duration::from_seconds( 0.2 );
      marker_array.markers.push_back( line_marker );
    }
    marker_array.markers.push_back( heading_marker );
  }
  auto closed_border = participant_set.validity_area.points;
  if( closed_border.size() > 0 )
  {
    closed_border.push_back( closed_border.front() );
    auto boundary_marker = primitives::create_line_marker( closed_border, "boundary", 999, 0.2, colors::green, offset );
    marker_array.markers.push_back( boundary_marker );
  }
  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::Trajectory& trajectory, const Offset& offset )
{
  MarkerArray marker_array;

  // Create the line marker for the trajectory
  auto line_marker = primitives::create_flat_line_marker( trajectory.states, "decision", trajectory.request_id, 1.8, colors::soft_blue,
                                                          offset );
  marker_array.markers.push_back( line_marker );

  // Determine the position for the label
  size_t num_states = trajectory.states.size();
  if( num_states > 0 )
  {
    const auto& first_state = trajectory.states[0];
    double      text_size   = 0.5;
    Color       text_color  = colors::blue;
    std::string ns          = "trajectory_label";

    double forward_distance = 2.0; // 2 meters forward
    double right_distance   = 2.0; // 2 meters to the right

    double text_x_position = first_state.x + forward_distance * cos( first_state.yaw_angle )
                           - right_distance * sin( first_state.yaw_angle );

    double text_y_position = first_state.y + forward_distance * sin( first_state.yaw_angle )
                           + right_distance * cos( first_state.yaw_angle );

    // Create the text markers
    MarkerArray text_markers = primitives::create_text_marker( text_x_position, text_y_position, trajectory.label, text_size, text_color,
                                                               ns, offset, first_state.yaw_angle );

    // Combine the markers from both arrays
    marker_array.markers.insert( marker_array.markers.end(), text_markers.markers.begin(), text_markers.markers.end() );
  }

  return marker_array;
}

// Conversion function for odometry to markers
MarkerArray
to_marker_array( const adore_ros2_msgs::msg::VehicleStateDynamic& msg, const Offset& offset )
{
  MarkerArray marker_array;

  auto ego_vehicle_marker = primitives::create_3d_object_marker( 0, 0,
                                                                 0.0, // Z height
                                                                 1,   // scale
                                                                 0.0, "ego_vehicle", 0, colors::blue, "low_poly_ngc_model.dae",
                                                                 { 0.0, 0.0 } ); // Create a rectangle marker for the ego vehicle

  ego_vehicle_marker.frame_locked    = true;
  ego_vehicle_marker.header.frame_id = "ego_vehicle";

  ego_vehicle_marker.mesh_use_embedded_materials = true;
  marker_array.markers.push_back( ego_vehicle_marker );
  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::TrafficSignals& traffic_signals, const Offset& offset )
{
  MarkerArray marker_array;

  for( size_t i = 0; i < traffic_signals.signals.size(); ++i )
  {
    const auto& signal = traffic_signals.signals[i];

    // Rectangle (signal housing)
    Marker rect_marker = primitives::create_rectangle_marker( signal.x, signal.y, 0.1, 0.7, 2.4, 0.2, 0.0, "traffic_signal", i,
                                                              colors::black, offset );
    marker_array.markers.push_back( rect_marker );

    // Circle markers for the lights
    const double circle_radius         = 0.6;
    const double offset_between_lights = 0.75; // Spacing between centers of lights

    // Red light
    Marker red_light = primitives::create_sphere_marker( signal.x, signal.y + offset_between_lights, 0.12, circle_radius,
                                                         "traffic_signal_red", i,
                                                         ( signal.state == adore_ros2_msgs::msg::TrafficSignal::RED ) ? colors::red
                                                                                                                      : colors::gray,
                                                         offset );
    marker_array.markers.push_back( red_light );

    // Yellow light
    Marker yellow_light = primitives::create_sphere_marker( signal.x, signal.y, 0.12, circle_radius, "traffic_signal_yellow", i,
                                                            ( signal.state == adore_ros2_msgs::msg::TrafficSignal::YELLOW ) ? colors::yellow
                                                                                                                            : colors::gray,
                                                            offset );
    marker_array.markers.push_back( yellow_light );

    // Green light
    Marker green_light = primitives::create_sphere_marker( signal.x, signal.y - offset_between_lights, 0.12, circle_radius,
                                                           "traffic_signal_green", i,
                                                           ( signal.state == adore_ros2_msgs::msg::TrafficSignal::GREEN ) ? colors::green
                                                                                                                          : colors::gray,
                                                           offset );
    marker_array.markers.push_back( green_light );
  }

  return marker_array;
}

// Conversion function for Waypoints message (just showing the points)
MarkerArray
to_marker_array( const adore_ros2_msgs::msg::Waypoints& waypoints_msg, const Offset& offset )
{
  MarkerArray marker_array;

  // Iterate over each waypoint and create a sphere marker
  for( size_t i = 0; i < waypoints_msg.waypoints.size(); ++i )
  {
    const auto& point = waypoints_msg.waypoints[i];
    // Create a sphere marker at the waypoint location (using a default scale, e.g., 0.2)
    Marker sphere_marker = primitives::create_sphere_marker( point.x, point.y, point.z,
                                                             0.3,         // Scale (radius)
                                                             "waypoints", // Namespace
                                                             i,           // ID
                                                             colors::purple, offset );
    marker_array.markers.push_back( sphere_marker );
  }

  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::VisualizableObject& msg, const Offset& offset )
{
  MarkerArray marker_array;

  auto object_marker = primitives::create_3d_object_marker( msg.x, msg.y,
                                                            msg.z, // Z height
                                                            1,     // scale
                                                            msg.yaw, msg.model, 0, colors::blue, msg.model,
                                                            offset ); // Create a rectangle marker for the ego vehicle

  object_marker.frame_locked    = true;
  object_marker.header.frame_id = "visualization_offset";

  object_marker.mesh_use_embedded_materials = true;
  marker_array.markers.push_back( object_marker );
  return marker_array;
}

MarkerArray
to_marker_array( const adore_ros2_msgs::msg::CautionZone& caution_zone, const Offset& offset )
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

  Marker line_marker = primitives::create_flat_line_marker( polygon_points, "caution_zone", 0, 0.5, colors::soft_orange, offset );
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
  size_t num_points  = polygon_points.size();
  centroid_x        /= num_points;
  centroid_y        /= num_points;

  MarkerArray text_markers = primitives::create_text_marker( centroid_x, centroid_y, caution_zone.label,
                                                             1.0,            // Text size
                                                             colors::orange, // Text color
                                                             caution_zone.label, offset );
  marker_array.markers.insert( marker_array.markers.end(), text_markers.markers.begin(), text_markers.markers.end() );

  return marker_array;
}
} // namespace conversions
} // namespace visualizer
} // namespace adore
