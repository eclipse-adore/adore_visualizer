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


std::optional<Image>
load_image( const std::string& image_file_path )
{
  cv::Mat loaded_image = cv::imread( image_file_path );

  if( loaded_image.empty() )
  {
    std::cerr << "Visualizer could not load the requeted image, path (" << image_file_path << ") is invalid" << std::endl;
    return {};
  }

  std_msgs::msg::Header header;  // optional, set timestamp/frame_id if needed
  // header.stamp = now();

  header.frame_id = "camera";
  sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", loaded_image).toImageMsg();

  return std::optional<Image> { *msg };
}

} // namespace primitives
} // namespace visualizer
} // namespace adore
