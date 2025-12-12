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

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "adore_ros2_msgs/msg/caution_zone.hpp"
#include "adore_ros2_msgs/msg/goal_point.hpp"
#include "adore_ros2_msgs/msg/map.hpp"
#include "adore_ros2_msgs/msg/map_point.hpp"
#include "adore_ros2_msgs/msg/route.hpp"
#include "adore_ros2_msgs/msg/safety_corridor.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "adore_ros2_msgs/msg/traffic_signals.hpp"
#include "adore_ros2_msgs/msg/trajectory.hpp"
#include "adore_ros2_msgs/msg/vehicle_state_dynamic.hpp"
#include "adore_ros2_msgs/msg/visualizable_object.hpp"
#include "adore_ros2_msgs/msg/waypoints.hpp"
#include <adore_map/lat_long_conversions.hpp>

#include "color_palette.hpp"
#include "visualizable_traits.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <nlohmann/json.hpp>

namespace conversions = adore::visualizer::conversions;
namespace colors      = adore::visualizer::colors;
using adore::visualizer::has_geo_json_conversion;
using adore::visualizer::has_marker_array_conversion;
using adore::visualizer::has_nav_sat_fix_conversion;

using json = nlohmann::json;

// -----------------------------------------------------------------------------
// SafetyCorridor
// -----------------------------------------------------------------------------

TEST( VisualizerConversions, safety_corridor_to_marker_array_creates_two_borders )
{
  adore_ros2_msgs::msg::SafetyCorridor corridor;

  geometry_msgs::msg::Point p1;
  p1.x = 0.0;
  p1.y = 0.0;

  geometry_msgs::msg::Point p2;
  p2.x = 10.0;
  p2.y = 0.0;

  geometry_msgs::msg::Point p3;
  p3.x = 0.0;
  p3.y = 2.0;

  geometry_msgs::msg::Point p4;
  p4.x = 10.0;
  p4.y = 2.0;

  corridor.left_border.push_back( p1 );
  corridor.left_border.push_back( p2 );
  corridor.right_border.push_back( p3 );
  corridor.right_border.push_back( p4 );

  auto marker_array = conversions::to_marker_array( corridor );

  ASSERT_EQ( marker_array.markers.size(), 2u );

  const auto& left_marker  = marker_array.markers[0];
  const auto& right_marker = marker_array.markers[1];

  EXPECT_EQ( left_marker.ns, "safety_left_border" );
  EXPECT_EQ( right_marker.ns, "safety_right_border" );


  EXPECT_DOUBLE_EQ( left_marker.scale.x, 0.6 );
  EXPECT_DOUBLE_EQ( right_marker.scale.x, 0.6 );
}

// -----------------------------------------------------------------------------
// Map
// -----------------------------------------------------------------------------

TEST( VisualizerConversions, map_to_marker_array_creates_lane_markers )
{
  adore_ros2_msgs::msg::Map map_msg;

  map_msg.roads.resize( 1 );
  auto& road = map_msg.roads[0];

  road.lanes.resize( 1 );
  auto& lane = road.lanes[0];

  lane.id = 7;

  adore_ros2_msgs::msg::MapPoint inner1;
  inner1.x = 0.0;
  inner1.y = 0.0;

  adore_ros2_msgs::msg::MapPoint inner2;
  inner2.x = 10.0;
  inner2.y = 0.0;

  adore_ros2_msgs::msg::MapPoint outer1;
  outer1.x = 0.0;
  outer1.y = 3.0;

  adore_ros2_msgs::msg::MapPoint outer2;
  outer2.x = 10.0;
  outer2.y = 3.0;

  adore_ros2_msgs::msg::MapPoint center1;
  center1.x = 0.0;
  center1.y = 1.5;

  adore_ros2_msgs::msg::MapPoint center2;
  center2.x = 10.0;
  center2.y = 1.5;

  lane.inner_points.push_back( inner1 );
  lane.inner_points.push_back( inner2 );
  lane.outer_points.push_back( outer1 );
  lane.outer_points.push_back( outer2 );
  lane.center_points.push_back( center1 );
  lane.center_points.push_back( center2 );

  auto marker_array = conversions::to_marker_array( map_msg );

  // inner, outer, center, road => 4 markers per lane
  ASSERT_EQ( marker_array.markers.size(), 4u );

  const auto& inner_marker  = marker_array.markers[0];
  const auto& outer_marker  = marker_array.markers[1];
  const auto& center_marker = marker_array.markers[2];
  const auto& road_marker   = marker_array.markers[3];

  EXPECT_EQ( inner_marker.ns, "inner" );
  EXPECT_EQ( outer_marker.ns, "outer" );
  EXPECT_EQ( center_marker.ns, "center" );
  EXPECT_EQ( road_marker.ns, "road" );

  EXPECT_EQ( inner_marker.id, lane.id );
  EXPECT_EQ( outer_marker.id, lane.id );
  EXPECT_EQ( center_marker.id, lane.id );
  EXPECT_EQ( road_marker.id, lane.id );


  // Lifetime explicitly set to 5 seconds
  EXPECT_EQ( inner_marker.lifetime.sec, 5 );
  EXPECT_EQ( outer_marker.lifetime.sec, 5 );
  EXPECT_EQ( center_marker.lifetime.sec, 5 );
  EXPECT_EQ( road_marker.lifetime.sec, 5 );
}

// -----------------------------------------------------------------------------
// Route (marker array)
// -----------------------------------------------------------------------------

TEST( VisualizerConversions, route_to_marker_array_creates_route_line )
{
  adore_ros2_msgs::msg::Route route;

  adore_ros2_msgs::msg::MapPoint p1;
  p1.x = 0.0;
  p1.y = 0.0;

  adore_ros2_msgs::msg::MapPoint p2;
  p2.x = 5.0;
  p2.y = 1.0;

  route.center_points.push_back( p1 );
  route.center_points.push_back( p2 );

  auto marker_array = conversions::to_marker_array( route );

  ASSERT_EQ( marker_array.markers.size(), 1u );
  const auto& marker = marker_array.markers.front();

  EXPECT_EQ( marker.ns, "route" );
  EXPECT_EQ( marker.id, 0 );

  EXPECT_DOUBLE_EQ( marker.scale.x, 0.2 );
  EXPECT_EQ( marker.points.size(), route.center_points.size() );
}

// -----------------------------------------------------------------------------
// GoalPoint (marker array)
// -----------------------------------------------------------------------------

TEST( VisualizerConversions, goal_point_to_marker_array_finish_line_and_label )
{
  adore_ros2_msgs::msg::GoalPoint goal;
  goal.x_position = 10.0;
  goal.y_position = 20.0;

  auto marker_array = conversions::to_marker_array( goal );

  // 4x4 grid from create_finish_line_marker -> 16 cubes,
  // plus one text marker "Goal"
  ASSERT_EQ( marker_array.markers.size(), 17u );

  std::size_t finish_count = 0;
  std::size_t text_count   = 0;

  for( const auto& marker : marker_array.markers )
  {
    if( marker.ns == "finish_line" )
    {
      ++finish_count;

      EXPECT_NEAR( marker.pose.position.z, 0.3, 1e-9 );
      EXPECT_DOUBLE_EQ( marker.scale.x, 0.5 );
      EXPECT_DOUBLE_EQ( marker.scale.y, 0.5 );
      EXPECT_DOUBLE_EQ( marker.scale.z, 0.01 );
    }
    else if( marker.ns == "goal_text" )
    {
      ++text_count;

      EXPECT_EQ( marker.text, "Goal" );
    }
  }

  EXPECT_EQ( finish_count, 16u );
  EXPECT_EQ( text_count, 1u );
}

// -----------------------------------------------------------------------------
// TrafficParticipantSet
// -----------------------------------------------------------------------------

TEST( VisualizerConversions, traffic_participant_set_creates_object_velocity_heading_route_and_boundary )
{
  adore_ros2_msgs::msg::TrafficParticipantSet set_msg;

  // One participant, non-controllable, with predicted trajectory and route
  set_msg.data.resize( 1 );
  auto& participant = set_msg.data[0];

  participant.participant_data.tracking_id    = 42;
  participant.participant_data.v2x_station_id = 0; // non-controllable

  // Motion state
  auto& state     = participant.participant_data.motion_state;
  state.x         = 1.0;
  state.y         = 2.0;
  state.vx        = 3.0;
  state.yaw_angle = 0.0; // Heading along +x

  // Physical parameters
  participant.participant_data.physical_parameters.body_length = 4.0;
  participant.participant_data.physical_parameters.body_width  = 1.8;
  participant.participant_data.physical_parameters.body_height = 1.5;

  // Goal point (use x >= 0.01 to take the purple color branch)
  participant.participant_data.goal_point.x = 1.0;

  // Predicted trajectory
  participant.participant_data.predicted_trajectory.states.resize( 3 );
  participant.participant_data.predicted_trajectory.states[0].x = 1.0;
  participant.participant_data.predicted_trajectory.states[0].y = 2.0;
  participant.participant_data.predicted_trajectory.states[1].x = 2.0;
  participant.participant_data.predicted_trajectory.states[1].y = 3.0;
  participant.participant_data.predicted_trajectory.states[2].x = 3.0;
  participant.participant_data.predicted_trajectory.states[2].y = 4.0;

  // Route (center line)
  adore_ros2_msgs::msg::MapPoint r1;
  r1.x = 0.0;
  r1.y = 0.0;
  adore_ros2_msgs::msg::MapPoint r2;
  r2.x = 1.0;
  r2.y = 0.0;
  participant.participant_data.route.center_points.push_back( r1 );
  participant.participant_data.route.center_points.push_back( r2 );

  // Validity area boundary
  adore_ros2_msgs::msg::Point2d b1;
  b1.x = 0.0;
  b1.y = 0.0;
  adore_ros2_msgs::msg::Point2d b2;
  b2.x = 5.0;
  b2.y = 0.0;
  adore_ros2_msgs::msg::Point2d b3;
  b3.x = 5.0;
  b3.y = 5.0;

  set_msg.validity_area.points.push_back( b1 );
  set_msg.validity_area.points.push_back( b2 );
  set_msg.validity_area.points.push_back( b3 );

  auto marker_array = conversions::to_marker_array( set_msg );

  // Expected markers:
  //  - rectangle object marker
  //  - velocity arrow
  //  - predicted trajectory flat line
  //  - heading line
  //  - route line
  //  - boundary line
  ASSERT_EQ( marker_array.markers.size(), 6u );

  // Object marker
  const auto& object_marker = marker_array.markers[0];
  EXPECT_EQ( object_marker.ns, "traffic_participant" );
  EXPECT_EQ( object_marker.id, participant.participant_data.tracking_id );

  EXPECT_EQ( object_marker.lifetime.sec, 1 );

  // Velocity marker
  const auto& velocity_marker = marker_array.markers[1];
  EXPECT_EQ( velocity_marker.ns, "participant_velocity" );
  EXPECT_EQ( velocity_marker.id, participant.participant_data.tracking_id + 100 );

  EXPECT_EQ( velocity_marker.lifetime.sec, 1 );

  // Trajectory marker
  const auto& traj_marker = marker_array.markers[2];
  EXPECT_EQ( traj_marker.ns, "decision" );
  EXPECT_EQ( traj_marker.id, participant.participant_data.tracking_id + 1000000 );

  EXPECT_EQ( traj_marker.lifetime.sec, 0 ); // 0.2 seconds stored in nanoseconds, sec part is 0

  // Heading marker
  const auto& heading_marker = marker_array.markers[3];
  EXPECT_EQ( heading_marker.ns, "participant_heading" );
  EXPECT_EQ( heading_marker.id, participant.participant_data.tracking_id + 10000 );

  EXPECT_EQ( heading_marker.lifetime.sec, 1 );

  // Route marker
  const auto& route_marker = marker_array.markers[4];
  EXPECT_EQ( route_marker.ns, "route" );
  EXPECT_EQ( route_marker.id, 2000000 + participant.participant_data.tracking_id );


  // Boundary marker
  const auto& boundary_marker = marker_array.markers[5];
  EXPECT_EQ( boundary_marker.ns, "boundary" );
  EXPECT_EQ( boundary_marker.id, 999 );
}

// -----------------------------------------------------------------------------
// Trajectory (marker array)
// -----------------------------------------------------------------------------

TEST( VisualizerConversions, empty_trajectory_yields_empty_marker_array )
{
  adore_ros2_msgs::msg::Trajectory trajectory;

  auto marker_array = conversions::to_marker_array( trajectory );
  EXPECT_TRUE( marker_array.markers.empty() );
}

TEST( VisualizerConversions, trajectory_to_marker_array_uses_flat_line_marker )
{
  adore_ros2_msgs::msg::Trajectory trajectory;
  trajectory.request_id = 123;

  // Fill a simple polyline with three states.
  trajectory.states.resize( 3 );
  trajectory.states[0].x = 0.0;
  trajectory.states[0].y = 0.0;

  trajectory.states[1].x = 5.0;
  trajectory.states[1].y = 0.0;

  trajectory.states[2].x = 5.0;
  trajectory.states[2].y = 1.0;

  auto marker_array = conversions::to_marker_array( trajectory );
  ASSERT_EQ( marker_array.markers.size(), 1u );

  const auto& marker = marker_array.markers.front();

  EXPECT_EQ( marker.ns, "decision" );
  EXPECT_EQ( marker.id, trajectory.request_id );


  // create_flat_line_marker builds 2 triangles per segment => 6 vertices/segment.
  const std::size_t expected_vertices = 6 * ( trajectory.states.size() - 1 );
  EXPECT_EQ( marker.points.size(), expected_vertices );
}

// -----------------------------------------------------------------------------
// VehicleStateDynamic (marker array)
// -----------------------------------------------------------------------------

TEST( VisualizerConversions, vehicle_state_dynamic_to_marker_array_creates_ego_mesh )
{
  adore_ros2_msgs::msg::VehicleStateDynamic state;
  state.header.frame_id = "ego_frame";

  auto marker_array = conversions::to_marker_array( state );
  ASSERT_EQ( marker_array.markers.size(), 1u );

  const auto& marker = marker_array.markers.front();

  EXPECT_EQ( marker.ns, "ego_vehicle" );
  EXPECT_EQ( marker.header.frame_id, state.header.frame_id );


  ASSERT_GE( marker.mesh_resource.size(), 4u );
  EXPECT_EQ( marker.mesh_resource.substr( marker.mesh_resource.size() - 4 ), ".dae" );

  EXPECT_TRUE( marker.frame_locked );
  EXPECT_TRUE( marker.mesh_use_embedded_materials );
}

// -----------------------------------------------------------------------------
// TrafficSignals
// -----------------------------------------------------------------------------

TEST( VisualizerConversions, traffic_signals_create_housing_and_three_lights )
{
  adore_ros2_msgs::msg::TrafficSignals signals_msg;
  signals_msg.signals.resize( 1 );
  auto& signal = signals_msg.signals[0];

  signal.x     = 1.0;
  signal.y     = 2.0;
  signal.state = adore_ros2_msgs::msg::TrafficSignal::RED;

  auto marker_array = conversions::to_marker_array( signals_msg );

  ASSERT_EQ( marker_array.markers.size(), 4u );

  const auto& housing      = marker_array.markers[0];
  const auto& red_light    = marker_array.markers[1];
  const auto& yellow_light = marker_array.markers[2];
  const auto& green_light  = marker_array.markers[3];

  EXPECT_EQ( housing.ns, "traffic_signal" );


  EXPECT_EQ( red_light.ns, "traffic_signal_red" );
  EXPECT_EQ( yellow_light.ns, "traffic_signal_yellow" );
  EXPECT_EQ( green_light.ns, "traffic_signal_green" );

  // Check that red is lit (not gray) and green is gray in RED state.
  EXPECT_GT( red_light.color.a, 0.0 );
  EXPECT_NE( red_light.color.r, yellow_light.color.r ); // some difference vs gray/yellow
}

// -----------------------------------------------------------------------------
// Waypoints
// -----------------------------------------------------------------------------

TEST( VisualizerConversions, waypoints_to_marker_array_creates_spheres )
{
  adore_ros2_msgs::msg::Waypoints waypoints_msg;

  adore_ros2_msgs::msg::Point2d w1;
  w1.x = 0.0;
  w1.y = 0.0;

  adore_ros2_msgs::msg::Point2d w2;
  w2.x = 1.0;
  w2.y = 2.0;

  waypoints_msg.waypoints.push_back( w1 );
  waypoints_msg.waypoints.push_back( w2 );

  auto marker_array = conversions::to_marker_array( waypoints_msg );

  ASSERT_EQ( marker_array.markers.size(), waypoints_msg.waypoints.size() );

  for( std::size_t i = 0; i < waypoints_msg.waypoints.size(); ++i )
  {
    const auto& marker   = marker_array.markers[i];
    const auto& waypoint = waypoints_msg.waypoints[i];

    EXPECT_EQ( marker.ns, "waypoints" );
    EXPECT_EQ( marker.id, static_cast<int>( i ) );

    EXPECT_DOUBLE_EQ( marker.pose.position.x, waypoint.x );
    EXPECT_DOUBLE_EQ( marker.pose.position.y, waypoint.y );
    EXPECT_DOUBLE_EQ( marker.scale.x, 0.3 );
  }
}

// -----------------------------------------------------------------------------
// VisualizableObject
// -----------------------------------------------------------------------------

TEST( VisualizerConversions, visualizable_object_to_marker_array_uses_model_name )
{
  adore_ros2_msgs::msg::VisualizableObject obj;
  obj.x     = 3.0;
  obj.y     = 4.0;
  obj.z     = 1.0;
  obj.yaw   = 0.5;
  obj.model = "test_model.dae";

  auto marker_array = conversions::to_marker_array( obj );
  ASSERT_EQ( marker_array.markers.size(), 1u );

  const auto& marker = marker_array.markers.front();

  EXPECT_EQ( marker.ns, obj.model );

  EXPECT_EQ( marker.mesh_resource, "http://localhost:8080/assets/3d_models/test_model.dae" );
  EXPECT_TRUE( marker.frame_locked );
  EXPECT_TRUE( marker.mesh_use_embedded_materials );
}

// -----------------------------------------------------------------------------
// CautionZone
// -----------------------------------------------------------------------------

TEST( VisualizerConversions, empty_caution_zone_polygon_yields_empty_array )
{
  adore_ros2_msgs::msg::CautionZone zone;
  auto                              marker_array = conversions::to_marker_array( zone );
  EXPECT_TRUE( marker_array.markers.empty() );
}

TEST( VisualizerConversions, caution_zone_polygon_creates_line_and_label )
{
  adore_ros2_msgs::msg::CautionZone zone;
  zone.label = "caution_1";

  adore_ros2_msgs::msg::Point2d p1;
  p1.x = 0.0;
  p1.y = 0.0;

  adore_ros2_msgs::msg::Point2d p2;
  p2.x = 2.0;
  p2.y = 0.0;

  adore_ros2_msgs::msg::Point2d p3;
  p3.x = 2.0;
  p3.y = 1.0;

  zone.polygon.points.push_back( p1 );
  zone.polygon.points.push_back( p2 );
  zone.polygon.points.push_back( p3 );

  auto marker_array = conversions::to_marker_array( zone );

  ASSERT_EQ( marker_array.markers.size(), 2u );

  const auto& line_marker = marker_array.markers[0];
  const auto& text_marker = marker_array.markers[1];

  EXPECT_EQ( line_marker.ns, "caution_zone" );


  EXPECT_EQ( text_marker.ns, zone.label );

  EXPECT_EQ( text_marker.text, zone.label );
}

// -----------------------------------------------------------------------------
// NavSatFix conversion
// -----------------------------------------------------------------------------

TEST( VisualizerConversions, vehicle_state_dynamic_to_nav_sat_fix_uses_lat_long_conversion )
{
  adore_ros2_msgs::msg::VehicleStateDynamic state;
  state.x = 450000.0;
  state.y = 5400000.0;

  auto fix = conversions::to_nav_sat_fix( state );

  auto lat_lon = adore::map::convert_utm_to_lat_lon( state.x, state.y, 32, "U" );

  ASSERT_EQ( lat_lon.size(), 2u );
  EXPECT_DOUBLE_EQ( fix.latitude, lat_lon[0] );
  EXPECT_DOUBLE_EQ( fix.longitude, lat_lon[1] );
}

// -----------------------------------------------------------------------------
// GeoJSON conversions
// -----------------------------------------------------------------------------

TEST( VisualizerConversions, goal_point_to_geojson_matches_lat_long_conversion )
{
  adore_ros2_msgs::msg::GoalPoint goal;
  goal.x_position = 450000.0;
  goal.y_position = 5400000.0;

  auto geo = conversions::to_geo_json( goal );
  auto j   = json::parse( geo.geojson );

  ASSERT_TRUE( j.contains( "features" ) );
  const auto& features = j.at( "features" );
  ASSERT_TRUE( features.is_array() );
  ASSERT_FALSE( features.empty() );

  const auto& feature  = features.at( 0 );
  const auto& geometry = feature.at( "geometry" );

  EXPECT_EQ( geometry.at( "type" ), "Point" );
  ASSERT_TRUE( geometry.at( "coordinates" ).is_array() );
  ASSERT_EQ( geometry.at( "coordinates" ).size(), 2u );

  const double lon = geometry.at( "coordinates" ).at( 0 );
  const double lat = geometry.at( "coordinates" ).at( 1 );

  auto lat_lon = adore::map::convert_utm_to_lat_lon( goal.x_position, goal.y_position, 32, "U" );
  ASSERT_EQ( lat_lon.size(), 2u );
  EXPECT_DOUBLE_EQ( lat, lat_lon[0] );
  EXPECT_DOUBLE_EQ( lon, lat_lon[1] );
}

TEST( VisualizerConversions, route_to_geojson_line_string_and_coordinates_match_conversion )
{
  adore_ros2_msgs::msg::Route route;

  adore_ros2_msgs::msg::MapPoint p1;
  p1.x = 0.0;
  p1.y = 0.0;

  adore_ros2_msgs::msg::MapPoint p2;
  p2.x = 10.0;
  p2.y = 0.0;

  route.center_points.push_back( p1 );
  route.center_points.push_back( p2 );

  auto geo = conversions::to_geo_json( route );
  auto j   = json::parse( geo.geojson );

  ASSERT_TRUE( j.contains( "features" ) );
  const auto& features = j.at( "features" );
  ASSERT_TRUE( features.is_array() );
  ASSERT_FALSE( features.empty() );

  const auto& feature  = features.at( 0 );
  const auto& geometry = feature.at( "geometry" );

  EXPECT_EQ( geometry.at( "type" ), "LineString" );
  ASSERT_TRUE( geometry.at( "coordinates" ).is_array() );
  ASSERT_EQ( geometry.at( "coordinates" ).size(), route.center_points.size() );

  for( std::size_t i = 0; i < route.center_points.size(); ++i )
  {
    const auto& coord = geometry.at( "coordinates" ).at( i );
    ASSERT_TRUE( coord.is_array() );
    ASSERT_EQ( coord.size(), 2u );

    const double lon = coord.at( 0 );
    const double lat = coord.at( 1 );

    auto lat_lon = adore::map::convert_utm_to_lat_lon( route.center_points[i].x, route.center_points[i].y, 32, "U" );
    ASSERT_EQ( lat_lon.size(), 2u );
    EXPECT_DOUBLE_EQ( lat, lat_lon[0] );
    EXPECT_DOUBLE_EQ( lon, lat_lon[1] );
  }
}

// -----------------------------------------------------------------------------
// Traits
// -----------------------------------------------------------------------------

TEST( VisualizerConversions, traits_detect_available_conversions )
{
  using adore_ros2_msgs::msg::GoalPoint;
  using adore_ros2_msgs::msg::Route;
  using adore_ros2_msgs::msg::Trajectory;
  using adore_ros2_msgs::msg::VehicleStateDynamic;

  static_assert( has_marker_array_conversion<GoalPoint>, "GoalPoint should be marker-array visualizable" );
  static_assert( has_marker_array_conversion<Route>, "Route should be marker-array visualizable" );
  static_assert( has_marker_array_conversion<Trajectory>, "Trajectory should be marker-array visualizable" );
  static_assert( has_nav_sat_fix_conversion<VehicleStateDynamic>, "VehicleStateDynamic should provide a NavSatFix conversion" );
  static_assert( has_geo_json_conversion<GoalPoint>, "GoalPoint should have GeoJSON conversion" );
  static_assert( has_geo_json_conversion<Route>, "Route should have GeoJSON conversion" );

  // Negative example to make sure the trait behaves as expected.
  static_assert( !has_marker_array_conversion<int>, "int must not be visualizable" );

  EXPECT_TRUE( (has_marker_array_conversion<GoalPoint>) );
  EXPECT_TRUE( (has_marker_array_conversion<Route>) );
  EXPECT_TRUE( (has_marker_array_conversion<Trajectory>) );
  EXPECT_TRUE( (has_nav_sat_fix_conversion<VehicleStateDynamic>) );
  EXPECT_TRUE( (has_geo_json_conversion<GoalPoint>) );
  EXPECT_TRUE( (has_geo_json_conversion<Route>) );
  EXPECT_FALSE( (has_marker_array_conversion<int>) );
}
