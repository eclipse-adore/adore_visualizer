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

// Function to create a checkered finish flag marker array
MarkerArray create_finish_line_marker( double x, double y, double square_size, const Offset& offset );

MarkerArray create_text_marker( double x, double y, const std::string& text, double size, const Color& color, const std::string& ns,
                                const Offset& offset, double rotation = 0 );
} // namespace primitives

} // namespace visualizer
} // namespace adore