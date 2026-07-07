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

#include "visualization_primitives.hpp"

#include <iostream>

namespace adore
{
namespace visualizer
{
namespace primitives
{


// Helper to create a rectangle (or cube) marker
Marker
create_rectangle_marker( double x, double y, double z, double length, double width, double height, double heading, const std::string& ns,
                         int id, const Color& color )
{
  Marker marker;
  marker.ns     = ns;
  marker.id     = id;
  marker.type   = Marker::CUBE;
  marker.action = Marker::ADD;

  // Set the position
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;

  // Set the orientation
  tf2::Quaternion q;
  q.setRPY( 0.0, 0.0, heading ); // Rotation around Z-axis (heading)
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  // Set the scale
  marker.scale.x = length;
  marker.scale.y = width;
  marker.scale.z = height;

  // Set the color
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];

  return marker;
}

// Helper to create a sphere marker
Marker
create_sphere_marker( double x, double y, double z, double scale, const std::string& ns, int id, const Color& color )
{
  Marker marker;
  marker.ns     = ns;
  marker.id     = id;
  marker.type   = Marker::SPHERE;
  marker.action = Marker::ADD;

  // Set the position
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;

  // Set the scale
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  // Set the color
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];

  return marker;
}

MarkerArray
create_finish_line_marker( double x, double y, double square_size )
{
  MarkerArray marker_array;

  int    grid_rows = 4; // Number of rows in the flag
  int    grid_cols = 4; // Number of columns in the flag
  double offset_x  = x;
  double offset_y  = y;

  // Loop through rows and columns to create the checkered pattern
  for( int row = 0; row < grid_rows; ++row )
  {
    for( int col = 0; col < grid_cols; ++col )
    {
      Marker marker;
      marker.ns     = "finish_line";
      marker.id     = row * grid_cols + col;
      marker.type   = Marker::CUBE;
      marker.action = Marker::ADD;

      // Set position for each square in the grid
      marker.pose.position.x    = offset_x + col * square_size;
      marker.pose.position.y    = offset_y + row * square_size;
      marker.pose.position.z    = 0.3;
      marker.pose.orientation.w = 1.0; // No rotation

      // Set the scale (size of each square)
      marker.scale.x = square_size;
      marker.scale.y = square_size;
      marker.scale.z = 0.01; // Thin flag for a 2D look

      // Alternate colors between black and white
      if( ( row + col ) % 2 == 0 )
      {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // Black
      }
      else
      {
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0; // White
      }

      // Add the marker to the marker array
      marker_array.markers.push_back( marker );
    }
  }

  return marker_array;
}

Marker
create_3d_object_marker( double x, double y, double z, double scale, double heading, const std::string& ns, int id, const Color& color,
                         const std::string& file_name )
{
  Marker marker;
  marker.ns     = ns;
  marker.id     = id;
  marker.type   = Marker::MESH_RESOURCE;
  marker.action = Marker::ADD;

  // Set the position
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;

  // Set the orientation
  tf2::Quaternion q;
  q.setRPY( 0.0, 0.0, heading ); // Rotation around Z-axis (heading)
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  // Set the scale
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  // Set the color
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];


  marker.mesh_resource = "http://localhost:8080/assets/3d_models/" + file_name;

  return marker;
}

Marker
create_text_marker( double x, double y, double z, const std::string& text, double size, const Color& color, const std::string& ns )
{
  Marker marker;
  marker.header.frame_id = "world";
  marker.ns              = ns;
  marker.id              = 0;
  marker.type            = Marker::TEXT_VIEW_FACING;
  marker.action          = Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;

  marker.scale.z = size;

  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];

  marker.text = text;

  marker.frame_locked = true;

  return marker;
}

void
transform_marker( Marker& marker, const geometry_msgs::msg::TransformStamped& transform )
{
  if( marker.type == visualization_msgs::msg::Marker::LINE_STRIP || marker.type == visualization_msgs::msg::Marker::LINE_LIST
      || marker.type == visualization_msgs::msg::Marker::POINTS || marker.type == visualization_msgs::msg::Marker::SPHERE_LIST
      || marker.type == visualization_msgs::msg::Marker::CUBE_LIST || marker.type == Marker::TRIANGLE_LIST )
  {
    for( auto& point : marker.points )
    {
      geometry_msgs::msg::PointStamped in_pt, out_pt;
      in_pt.header = marker.header;
      in_pt.point  = point;

      tf2::doTransform( in_pt, out_pt, transform );
      point = out_pt.point; // Now 'point' is in the new_frame_id coords
    }
  }
  else
  {

    geometry_msgs::msg::PoseStamped pose_in, pose_out;
    pose_in.header = marker.header;
    pose_in.pose   = marker.pose;

    tf2::doTransform( pose_in, pose_out, transform );

    marker.pose = pose_out.pose;
  }
}



MarkerArray
create_flagpole_marker( double x, double y, const std::string& ns, int id,
                        const Color& pole_color, const Color& flag_color,
                        const std::string& label )
{
  MarkerArray array;

  constexpr double POLE_HEIGHT = 4.0;
  constexpr double POLE_WIDTH  = 0.08;
  constexpr double FLAG_WIDTH  = 1.2;
  constexpr double FLAG_HEIGHT = 0.7;
  constexpr double FLAG_DEPTH  = 0.05;

  Marker pole;
  pole.ns                  = ns + "_pole";
  pole.id                  = id;
  pole.type                = Marker::CUBE;
  pole.action              = Marker::ADD;
  pole.pose.position.x     = x;
  pole.pose.position.y     = y;
  pole.pose.position.z     = POLE_HEIGHT / 2.0;
  pole.pose.orientation.w  = 1.0;
  pole.scale.x             = POLE_WIDTH;
  pole.scale.y             = POLE_WIDTH;
  pole.scale.z             = POLE_HEIGHT;
  pole.color.r             = pole_color[0];
  pole.color.g             = pole_color[1];
  pole.color.b             = pole_color[2];
  pole.color.a             = pole_color[3];
  pole.frame_locked        = true;
  pole.header.frame_id     = "world";
  array.markers.push_back( pole );

  Marker flag;
  flag.ns                  = ns + "_flag";
  flag.id                  = id;
  flag.type                = Marker::CUBE;
  flag.action              = Marker::ADD;
  flag.pose.position.x     = x + FLAG_WIDTH / 2.0 + POLE_WIDTH / 2.0;
  flag.pose.position.y     = y;
  flag.pose.position.z     = POLE_HEIGHT - FLAG_HEIGHT / 2.0;
  flag.pose.orientation.w  = 1.0;
  flag.scale.x             = FLAG_WIDTH;
  flag.scale.y             = FLAG_DEPTH;
  flag.scale.z             = FLAG_HEIGHT;
  flag.color.r             = flag_color[0];
  flag.color.g             = flag_color[1];
  flag.color.b             = flag_color[2];
  flag.color.a             = flag_color[3];
  flag.frame_locked        = true;
  flag.header.frame_id     = "world";
  array.markers.push_back( flag );

  if( !label.empty() )
  {
    Marker text           = create_text_marker( x, y, POLE_HEIGHT + FLAG_HEIGHT + 0.3,
                                                label, 0.7, pole_color, ns + "_label" );
    text.id               = id;
    text.frame_locked     = true;
    text.header.frame_id  = "world";
    array.markers.push_back( text );
  }

  return array;
}

} // namespace primitives
} // namespace visualizer
} // namespace adore
