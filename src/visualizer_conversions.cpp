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

namespace adore
{
namespace visualizer
{
namespace conversions
{
// Conversion function for time-based odometry buffer to a marker array using the templated line drawing primitive
MarkerArray
state_buffer_to_markers( const StateBuffer& state_buffer, const Offset& offset )
{
  MarkerArray marker_array;

  const auto& buffer = state_buffer.get();

  if( buffer.size() < 2 )
  {
    return marker_array; // Not enough points to create a line
  }

  // Iterate over the buffer to create lines between consecutive points
  for( size_t i = 1; i < buffer.size(); ++i )
  {
    Marker line_marker = primitives::create_line_marker( buffer[i - 1], buffer[i], "driven_path", i, 0.4, colors::blue, offset );

    marker_array.markers.push_back( line_marker );
  }

  return marker_array;
}

MarkerArray
safety_corridor_to_markers( const adore_ros2_msgs::msg::SafetyCorridor& safety_corridor, const Offset& offset )
{
  MarkerArray marker_array;

  auto line_marker_left = primitives::create_line_marker( safety_corridor.left_border, "safety_left_border", 0, 0.6, colors::red, offset );
  marker_array.markers.push_back( line_marker_left );

  auto line_marker_right = primitives::create_line_marker( safety_corridor.right_border, "safety_right_border", 0, 0.6, colors::red,
                                                           offset );
  marker_array.markers.push_back( line_marker_right );

  return marker_array;
}

MarkerArray
traffic_prediction_to_markers( const adore_ros2_msgs::msg::TrafficPrediction& traffic_prediction, const Offset& offset )
{
  MarkerArray marker_array;


  for( const auto& vehicle : traffic_prediction.traffic_prediction )
  {

    auto line_marker = primitives::create_line_marker( vehicle.trajectory_prediction.states, "traffic_prediction", 0, 0.6, colors::green,
                                                       offset );
    marker_array.markers.push_back( line_marker );
  }

  return marker_array;
}

MarkerArray
map_to_marker_array( const adore_ros2_msgs::msg::Map& local_map_msg, const Offset& offset )
{
  MarkerArray marker_array;

  for( const auto& road : local_map_msg.roads )
  {
    for( const auto& lane : road.lanes )
    {
      auto inner_marker  = primitives::create_line_marker( lane.inner_points, "inner", lane.id, 0.2, colors::pink, offset );
      auto outer_marker  = primitives::create_line_marker( lane.outer_points, "outer", lane.id, 0.2, colors::pink, offset );
      auto center_marker = primitives::create_line_marker( lane.center_points, "center", lane.id, 0.1, colors::gray, offset );
      marker_array.markers.push_back( inner_marker );
      marker_array.markers.push_back( outer_marker );
      marker_array.markers.push_back( center_marker );
    }
  }


  return marker_array;
}

MarkerArray
route_to_marker_array( const adore_ros2_msgs::msg::Route& route, const Offset& offset )
{

  MarkerArray marker_array;

  auto line_marker = primitives::create_line_marker( route.center_points, "route", 0, 0.2, colors::yellow, offset );
  marker_array.markers.push_back( line_marker );

  return marker_array;
}

MarkerArray
goal_to_marker_array( const adore_ros2_msgs::msg::GoalPoint& goal, const Offset& offset )
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
  Color       text_color = colors::cyan; // Assuming you have a colors::blue defined
  std::string ns         = "goal_text";

  // Position the text next to the finish line marker
  // Calculate the x and y positions for the text
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
traffic_participants_to_markers( const adore_ros2_msgs::msg::TrafficParticipantSet& participant_set, const Offset& offset )
{
  MarkerArray marker_array;

  for( const auto& participant : participant_set.data )
  {
    const auto& state   = participant.participant_data.motion_state;
    double      heading = state.yaw_angle; // + M_PI / 4;

    // Compute unit vector for heading
    double unit_vector_x = std::cos( heading ) + std::sin( heading );
    double unit_vector_y = -std::sin( heading ) + std::cos( heading );

    // Add rectangle marker for the participant
    double participant_length = 4.0;
    double participant_width  = 1.8;
    double participant_height = 1.0;
    if( participant.participant_data.shape.dimensions.size() >= 3 )
    {
      participant_length = participant.participant_data.shape.dimensions[0];
      participant_width  = participant.participant_data.shape.dimensions[1];
      participant_height = participant.participant_data.shape.dimensions[2];
    }

    auto rectangle_marker     = primitives::create_rectangle_marker( state.x, state.y,
                                                                     0.2,                                      // Z position for height
                                                                     participant_length,                       // Length
                                                                     participant_width,                        // Width
                                                                     participant_height,                       // Height (example)
                                                                     heading,                                  // Orientation
                                                                     "traffic_participant",                    // Namespace
                                                                     participant.participant_data.tracking_id, // ID
                                                                     colors::red, offset );
    rectangle_marker.lifetime = rclcpp::Duration::from_seconds( 0.2 ); // Add lifetime
    marker_array.markers.push_back( rectangle_marker );

    // Add velocity line marker
    geometry_msgs::msg::Point start;
    start.x = state.x;
    start.y = state.y;

    geometry_msgs::msg::Point end;
    end.x = start.x + state.vx;
    end.y = start.y + state.vy;

    static const int VEL_ID_OFFSET = 1000; // for separate id for velocity marker

    auto velocity_marker     = primitives::create_line_marker( start, end, "participant_velocity",
                                                               participant.participant_data.tracking_id + VEL_ID_OFFSET,
                                                               0.1,                   // Line width
                                                               colors::orange, offset // Light blue color with 50% opacity
        );
    velocity_marker.type     = velocity_marker.ARROW;
    velocity_marker.lifetime = rclcpp::Duration::from_seconds( 0.2 ); // Add lifetime
    marker_array.markers.push_back( velocity_marker );

    // Add heading line marker
    geometry_msgs::msg::Point heading_end;
    heading_end.x = start.x + unit_vector_x;
    heading_end.y = start.y + unit_vector_y;

    static const int HEADING_ID_OFFSET = 1000000; // for separate id for heading marker

    auto heading_marker     = primitives::create_line_marker( start, heading_end, "participant_heading",
                                                              participant.participant_data.tracking_id + HEADING_ID_OFFSET,
                                                              0.2, // Line width
                                                              colors::gray, offset );
    heading_marker.lifetime = rclcpp::Duration::from_seconds( 0.2 ); // Add lifetime

    if (participant.participant_data.predicted_trajectory.states.size() > 0) 
    {
      // Create the line marker for the trajectory
      auto line_marker = primitives::create_line_marker( participant.participant_data.predicted_trajectory.states, "decision", participant.participant_data.tracking_id, 0.3, colors::green, offset );
      marker_array.markers.push_back( line_marker );
    }
    marker_array.markers.push_back( heading_marker );
  }

  return marker_array;
}

MarkerArray
ignored_participants_to_markers( const adore_ros2_msgs::msg::TrafficParticipantSet& ignored_participant_set, const Offset& offset )
{
  MarkerArray marker_array;

  for( const auto& ignored : ignored_participant_set.data )
  {
    const auto& state   = ignored.participant_data.motion_state;
    double      heading = state.yaw_angle;

    // Add rectangle marker for the ignored participant
    auto rectangle_marker     = primitives::create_rectangle_marker( state.x, state.y,
                                                                     0.2,                                          // Z position for height
                                                                     ignored.participant_data.shape.dimensions[0], // Length
                                                                     ignored.participant_data.shape.dimensions[1], // Width
                                                                     ignored.participant_data.shape.dimensions[2], // Height
                                                                     heading,                                      // Orientation
                                                                     "traffic_participant",                        // Namespace
                                                                     ignored.participant_data.tracking_id,         // ID
                                                                     colors::gray, offset );
    rectangle_marker.lifetime = rclcpp::Duration::from_seconds( 0.2 ); // Add lifetime
    marker_array.markers.push_back( rectangle_marker );
  }

  return marker_array;
}

MarkerArray
trajectory_to_markers( const adore_ros2_msgs::msg::Trajectory& trajectory, const Offset& offset )
{
  MarkerArray marker_array;

  // Create the line marker for the trajectory
  auto line_marker = primitives::create_line_marker( trajectory.states, "decision", trajectory.request_id, 0.3, colors::green, offset );
  marker_array.markers.push_back( line_marker );

  // Determine the position for the label
  size_t num_states = trajectory.states.size();
  if( num_states > 0 )
  {
    const auto& first_state = trajectory.states[0];
    double      text_size   = 0.5;
    Color       text_color  = colors::green;
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
state_to_markers( const adore_ros2_msgs::msg::VehicleStateDynamic& msg, const Offset& offset )
{
  MarkerArray marker_array;

  // Create a rectangle marker for the ego vehicle
  auto ego_vehicle_marker                        = primitives::create_rectangle_marker( msg.x, msg.y,
                                                                                        0.0, // Z height
                                                                                        4.5, // Length
                                                                                        2.0, // Width
                                                                                        1.5, // Height
                                                                                        msg.yaw_angle, "ego_vehicle", 0, colors::blue,
                                                                                        offset // Red color with 80% opacity
                         );
  ego_vehicle_marker.mesh_use_embedded_materials = true;
  marker_array.markers.push_back( ego_vehicle_marker );

  return marker_array;
}

MarkerArray
traffic_signals_to_markers( const adore_ros2_msgs::msg::TrafficSignals& traffic_signals, const Offset& offset )
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

} // namespace conversions
} // namespace visualizer
} // namespace adore
