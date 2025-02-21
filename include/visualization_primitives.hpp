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
 *    Mikkel Skov Maarssø
 *    Marko Mizdrak

 ********************************************************************************/
#pragma once
#include <adore_ros2_msgs/msg/traffic_participant_set.hpp>

#include "characters.hpp"
#include "color_palette.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace adore
{
namespace visualizer
{

struct Offset
{
  double x;
  double y;
};

using Marker      = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

namespace primitives
{

// Helper to create a rectangle (or cube) marker
Marker create_rectangle_marker( double x, double y, double z, double length, double width, double height, double heading,
                                const std::string& ns, int id, const Color& color, const Offset& offset );

Marker create_3d_object_marker( double x, double y, double z, double scale, double heading, const std::string& ns, int id,
                                const Color& color, const std::string& file_name, const Offset& offset );

// Helper to create a sphere marker
Marker create_sphere_marker( double x, double y, double z, double scale, const std::string& ns, int id, const Color& color,
                             const Offset& offset );

// Template function to create a line marker (can accept any type with x, y fields)
template<typename PointType>
Marker
create_line_marker( const PointType& start, const PointType& end, const std::string& ns, int id, double scale, const Color& color,
                    const Offset& offset )
{
  Marker marker;
  marker.header.frame_id = "visualization_offset";
  marker.ns              = ns;
  marker.id              = id;
  marker.type            = Marker::LINE_STRIP;
  marker.action          = Marker::ADD;

  // Add start and end points
  geometry_msgs::msg::Point start_point, end_point;
  start_point.x = start.x - offset.x;
  start_point.y = start.y - offset.y;
  start_point.z = 0.0; // Assuming a 2D point, z can be set to 0.

  end_point.x = end.x - offset.x;
  end_point.y = end.y - offset.y;
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
create_line_marker( const IterablePoints& points, const std::string& ns, int id, double scale, const Color& color, const Offset& offset )
{
  Marker marker;
  marker.header.frame_id = "visualization_offset";
  marker.ns              = ns;
  marker.id              = id;
  marker.type            = Marker::LINE_STRIP;
  marker.action          = Marker::ADD;

  for( const auto& point : points )
  {
    // Add start and end points
    geometry_msgs::msg::Point marker_point;
    marker_point.x = point.x - offset.x;
    marker_point.y = point.y - offset.y;
    marker_point.z = 0.0; // Assuming a 2D point, z can be set to 0.

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
create_flat_line_marker( const IterablePoints& points, const std::string& ns, int id, double width, const Color& color,
                         const Offset& offset )
{
  Marker marker;
  marker.header.frame_id = "visualization_offset";
  marker.ns              = ns;
  marker.id              = id;
  marker.type            = Marker::TRIANGLE_LIST; // Using triangles to form quads
  marker.action          = Marker::ADD;

  // Set the color
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];

  if( points.size() < 2 )
  {
    return marker; // Not enough points to form a line
  }

  // Iterate through points to create a quad between each pair
  auto prev = points.begin();
  for( auto next = std::next( prev ); next != points.end(); ++prev, ++next )
  {
    double dx     = next->x - prev->x;
    double dy     = next->y - prev->y;
    double length = std::sqrt( dx * dx + dy * dy );

    if( length == 0 )
      continue; // Avoid division by zero

    // Compute perpendicular unit vector for width control
    double ux = -( dy / length ) * ( width / 2.0 );
    double uy = ( dx / length ) * ( width / 2.0 );

    // Define the four corner points of the quad (flat rectangle)
    geometry_msgs::msg::Point p1, p2, p3, p4;
    p1.x = prev->x - ux - offset.x;
    p1.y = prev->y - uy - offset.y;
    p1.z = 0.0;
    p2.x = prev->x + ux - offset.x;
    p2.y = prev->y + uy - offset.y;
    p2.z = 0.0;
    p3.x = next->x - ux - offset.x;
    p3.y = next->y - uy - offset.y;
    p3.z = 0.0;
    p4.x = next->x + ux - offset.x;
    p4.y = next->y + uy - offset.y;
    p4.z = 0.0;

    // Define two triangles per quad
    marker.points.push_back( p1 );
    marker.points.push_back( p2 );
    marker.points.push_back( p3 );

    marker.points.push_back( p2 );
    marker.points.push_back( p4 );
    marker.points.push_back( p3 );
  }

  return marker;
}

// Function to create a checkered finish flag marker array
MarkerArray create_finish_line_marker( double x, double y, double square_size, const Offset& offset );

MarkerArray create_text_marker( double x, double y, const std::string& text, double size, const Color& color, const std::string& ns,
                                const Offset& offset, double rotation = 0 );
} // namespace primitives

} // namespace visualizer
} // namespace adore
