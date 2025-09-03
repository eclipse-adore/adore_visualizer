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

#pragma once
#include <Eigen/Dense>

#include <adore_ros2_msgs/msg/traffic_participant_set.hpp>

#include "color_palette.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace adore
{
namespace visualizer
{

using Marker      = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

namespace primitives
{


// Helper to create a rectangle (or cube) marker
Marker create_rectangle_marker( double x, double y, double z, double length, double width, double height, double heading,
                                const std::string& ns, int id, const Color& color );

Marker create_3d_object_marker( double x, double y, double z, double scale, double heading, const std::string& ns, int id,
                                const Color& color, const std::string& file_name );

// Helper to create a sphere marker
Marker create_sphere_marker( double x, double y, double z, double scale, const std::string& ns, int id, const Color& color );


// Function to create a checkered finish flag marker array
MarkerArray create_finish_line_marker( double x, double y, double square_size );

Marker create_text_marker( double x, double y, double z, const std::string& text, double size, const Color& color, const std::string& ns );

void transform_marker( Marker& marker, const geometry_msgs::msg::TransformStamped& transform );

// Template function to create a line marker (can accept any type with x, y fields)
template<typename PointType>
Marker
create_line_marker( const PointType& start, const PointType& end, const std::string& ns, int id, double scale, const Color& color )
{
  Marker marker;
  marker.ns     = ns;
  marker.id     = id;
  marker.type   = Marker::LINE_STRIP;
  marker.action = Marker::ADD;

  // Add start and end points
  geometry_msgs::msg::Point start_point, end_point;
  start_point.x = start.x;
  start_point.y = start.y;
  start_point.z = 0.0; // Assuming a 2D point, z can be set to 0.

  end_point.x = end.x;
  end_point.y = end.y;
  end_point.z = 0.0;

  marker.points.push_back( start_point );
  marker.points.push_back( end_point );

  // Set the scale (width of the line)
  marker.scale.x = scale;

  // Set the color
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];

  return marker;
}

// Template function to create a line marker (can accept any type with x, y fields)
template<typename IterablePoints>
Marker
create_line_marker( const IterablePoints& points, const std::string& ns, int id, double scale, const Color& color )
{
  Marker marker;
  marker.ns     = ns;
  marker.id     = id;
  marker.type   = Marker::LINE_STRIP;
  marker.action = Marker::ADD;

  for( const auto& point : points )
  {
    // Add start and end points
    geometry_msgs::msg::Point marker_point;
    marker_point.x = point.x;
    marker_point.y = point.y;
    marker_point.z = 0.5;

    marker.points.push_back( marker_point );
  }


  // Set the scale (width of the line)
  marker.scale.x = scale;

  // Set the color
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];

  return marker;
}

template<typename IterablePoints>
Marker
create_flat_line_marker( const IterablePoints& points, const std::string& ns, int id, double width, const Color& color )
{
  Marker marker;

  if( points.size() < 3 )
    return marker; // Not enough points to form a line.

  marker.ns     = ns;
  marker.id     = id;
  marker.type   = Marker::TRIANGLE_LIST; // Using triangles to form quads
  marker.action = Marker::ADD;

  // Set the color.
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];


  // Compute per-vertex offsets.
  // Each offset is an Eigen::Vector2d representing (x,y) offset for that vertex.
  std::vector<Eigen::Vector2d> vertex_offsets;
  vertex_offsets.reserve( points.size() );

  // For the first vertex, use the normal from the first segment.
  {
    auto            it     = points.begin();
    auto            nextIt = std::next( it );
    double          dx     = nextIt->x - it->x;
    double          dy     = nextIt->y - it->y;
    double          len    = std::sqrt( dx * dx + dy * dy );
    Eigen::Vector2d normal = ( len > 0 ) ? Eigen::Vector2d( -dy / len, dx / len ) : Eigen::Vector2d( 0, 0 );
    vertex_offsets.push_back( normal * ( width / 2.0 ) );
  }

  // For intermediate vertices, average the normals from the incoming and outgoing segments.
  for( auto it = std::next( points.begin() ); it != std::prev( points.end() ); ++it )
  {
    auto prevIt = std::prev( it );
    auto nextIt = std::next( it );

    double          dx1     = it->x - prevIt->x;
    double          dy1     = it->y - prevIt->y;
    double          len1    = std::sqrt( dx1 * dx1 + dy1 * dy1 );
    Eigen::Vector2d normal1 = ( len1 > 0 ) ? Eigen::Vector2d( -dy1 / len1, dx1 / len1 ) : Eigen::Vector2d( 0, 0 );

    double          dx2     = nextIt->x - it->x;
    double          dy2     = nextIt->y - it->y;
    double          len2    = std::sqrt( dx2 * dx2 + dy2 * dy2 );
    Eigen::Vector2d normal2 = ( len2 > 0 ) ? Eigen::Vector2d( -dy2 / len2, dx2 / len2 ) : Eigen::Vector2d( 0, 0 );

    Eigen::Vector2d avg_normal;
    if( normal1.norm() > 0 || normal2.norm() > 0 )
      avg_normal = ( normal1 + normal2 ).normalized() * ( width / 2.0 );
    else
      avg_normal = Eigen::Vector2d( 0, 0 );

    vertex_offsets.push_back( avg_normal );
  }

  // For the last vertex, use the normal from the last segment.
  {
    auto            it     = std::prev( points.end() );
    auto            prevIt = std::prev( it );
    double          dx     = it->x - prevIt->x;
    double          dy     = it->y - prevIt->y;
    double          len    = std::sqrt( dx * dx + dy * dy );
    Eigen::Vector2d normal = ( len > 0 ) ? Eigen::Vector2d( -dy / len, dx / len ) : Eigen::Vector2d( 0, 0 );
    vertex_offsets.push_back( normal * ( width / 2.0 ) );
  }

  // Now create quads between consecutive points using the computed per-vertex offsets.
  auto point_it = points.begin();
  for( size_t i = 0; i < points.size() - 1; i++ )
  {
    // Get the current and next points.
    auto current = *point_it++;
    auto next    = *point_it;

    // Retrieve the precomputed offsets for these vertices.
    const Eigen::Vector2d& off_current = vertex_offsets[i];
    const Eigen::Vector2d& off_next    = vertex_offsets[i + 1];

    // Define four corners of the quad.
    geometry_msgs::msg::Point p1, p2, p3, p4;
    p1.x = current.x - off_current.x();
    p1.y = current.y - off_current.y();
    p1.z = 0.5;
    p2.x = current.x + off_current.x();
    p2.y = current.y + off_current.y();
    p2.z = 0.5;
    p3.x = next.x - off_next.x();
    p3.y = next.y - off_next.y();
    p3.z = 0.5;
    p4.x = next.x + off_next.x();
    p4.y = next.y + off_next.y();
    p4.z = 0.5;

    // Create two triangles to form the quad.
    marker.points.push_back( p1 );
    marker.points.push_back( p2 );
    marker.points.push_back( p3 );

    marker.points.push_back( p2 );
    marker.points.push_back( p4 );
    marker.points.push_back( p3 );
  }

  return marker;
}

} // namespace primitives

} // namespace visualizer
} // namespace adore
