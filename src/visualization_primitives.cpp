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
                         int id, const Color& color, const Offset& offset )
{
  Marker marker;
  marker.header.frame_id = "visualization_offset";
  marker.ns              = ns;
  marker.id              = id;
  marker.type            = Marker::CUBE;
  marker.action          = Marker::ADD;

  // Set the position
  marker.pose.position.x = x - offset.x;
  marker.pose.position.y = y - offset.y;
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
create_sphere_marker( double x, double y, double z, double scale, const std::string& ns, int id, const Color& color, const Offset& offset )
{
  Marker marker;
  marker.header.frame_id = "visualization_offset";
  marker.ns              = ns;
  marker.id              = id;
  marker.type            = Marker::SPHERE;
  marker.action          = Marker::ADD;

  // Set the position
  marker.pose.position.x = x - offset.x;
  marker.pose.position.y = y - offset.y;
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
create_finish_line_marker( double x, double y, double square_size, const Offset& offset, const std::string& frame_id )
{
  MarkerArray marker_array;

  int    grid_rows = 4; // Number of rows in the flag
  int    grid_cols = 4; // Number of columns in the flag
  double offset_x  = x - offset.x;
  double offset_y  = y - offset.y;

  // Loop through rows and columns to create the checkered pattern
  for( int row = 0; row < grid_rows; ++row )
  {
    for( int col = 0; col < grid_cols; ++col )
    {
      Marker marker;
      marker.header.frame_id = frame_id;
      marker.ns              = "finish_line";
      marker.id              = row * grid_cols + col;
      marker.type            = Marker::CUBE;
      marker.action          = Marker::ADD;

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
                         const std::string& file_name, const Offset& offset )
{
  Marker marker;
  marker.header.frame_id = "visualization_offset"; // @TODO, this might cause future problems
  marker.ns              = ns;
  marker.id              = id;
  marker.type            = Marker::MESH_RESOURCE;
  marker.action          = Marker::ADD;

  // Set the position
  marker.pose.position.x = x - offset.x;
  marker.pose.position.y = y - offset.y;
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

MarkerArray
create_text_marker( double x, double y, const std::string& text, double size, const Color& color, const std::string& ns,
                    const Offset& offset, const std::string& frame_id, double rotation )
{
  MarkerArray marker_array;

  // Dimensions and spacing
  double pixel_size   = size / 5.0; // Size of each pixel
  double char_spacing = size * 1.2;

  int    marker_id        = 0;
  double current_x_offset = 0.0;

  // Precompute sin and cos of rotation angle
  double cos_rotation = std::cos( rotation );
  double sin_rotation = std::sin( rotation );

  for( char c : text )
  {
    // Get the bitmap for the character
    std::array<uint8_t, 7> bitmap          = { 0 };
    bool                   character_found = false;
    for( const auto& [key, value] : CHARACTER_MAP )
    {
      if( key == toupper( c ) )
      {
        bitmap          = value;
        character_found = true;
        break;
      }
    }
    if( !character_found )
    {
      current_x_offset += char_spacing; // Skip unrecognized characters
      continue;
    }

    // Loop through the 5x7 grid
    for( int row = 0; row < 7; ++row )
    {
      uint8_t row_bits = bitmap[row];
      for( int col = 0; col < 5; ++col )
      {
        if( row_bits & ( 1 << ( 4 - col ) ) ) // Check if the pixel is lit
        {
          Marker pixel_marker;
          pixel_marker.header.frame_id = frame_id;
          pixel_marker.ns              = ns;
          pixel_marker.id              = marker_id++;
          pixel_marker.type            = Marker::CUBE;
          pixel_marker.action          = Marker::ADD;

          // Original position relative to text origin
          double original_x = current_x_offset + col * pixel_size;
          double original_y = ( 6 - row ) * pixel_size;

          // Apply rotation around text origin
          double rotated_x = original_x * cos_rotation - original_y * sin_rotation;
          double rotated_y = original_x * sin_rotation + original_y * cos_rotation;

          // Set position for each pixel
          pixel_marker.pose.position.x = x - offset.x + rotated_x;
          pixel_marker.pose.position.y = y - offset.y + rotated_y;
          pixel_marker.pose.position.z = 0.0;

          // Set orientation (no rotation needed for individual pixels)
          pixel_marker.pose.orientation.x = 0.0;
          pixel_marker.pose.orientation.y = 0.0;
          pixel_marker.pose.orientation.z = 0.0;
          pixel_marker.pose.orientation.w = 1.0;

          // Set the scale (size of each pixel)
          pixel_marker.scale.x = pixel_size;
          pixel_marker.scale.y = pixel_size;
          pixel_marker.scale.z = 0.01; // Thin pixel for a 2D look

          // Set the color
          pixel_marker.color.r = color[0];
          pixel_marker.color.g = color[1];
          pixel_marker.color.b = color[2];
          pixel_marker.color.a = color[3];

          // Add the marker to the marker array
          marker_array.markers.push_back( pixel_marker );
        }
      }
    }

    // Advance current_x_offset for the next character
    current_x_offset += char_spacing;
  }

  return marker_array;
}


} // namespace primitives
} // namespace visualizer
} // namespace adore
