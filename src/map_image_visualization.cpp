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
#include "map_image_visualization.hpp"

namespace adore
{
namespace visualizer
{
namespace map_image
{
#include <iomanip> // For std::setprecision
#include <sstream>

// Helper function for downloading or loading the map image
std::optional<cv::Mat>
fetch_map_image( int map_tile_x, int map_tile_y, double tile_size, double map_size, int image_resolution,
                 const std::string &map_storage_path, bool networking_disabled )
{
  // Try loading the image from the local file system
  cv::Mat map_image = cv::imread( map_storage_path + "/" + std::to_string( map_tile_x ) + ":" + std::to_string( map_tile_y ) + ".jpg" );

  if( !map_image.empty() )
  {
    return map_image;
  }

  // If networking is disabled, we can't fetch the map from the server
  if( networking_disabled )
  {
    return std::nullopt;
  }

  // Base URL for WMS service
  std::string base_url = "https://opendata.lgln.niedersachsen.de/doorman/noauth/dop_wms?";

  // Calculate bounding box coordinates for the requested map tile
  double bottom_left_x = map_tile_x * tile_size - map_size / 2.0;
  double bottom_left_y = map_tile_y * tile_size - map_size / 2.0;
  double top_right_x   = map_tile_x * tile_size + tile_size + map_size / 2.0;
  double top_right_y   = map_tile_y * tile_size + tile_size + map_size / 2.0;

  // Construct the WMS request URL
  std::ostringstream bbox_stream;
  bbox_stream << std::fixed << std::setprecision( 6 ) << bottom_left_x << "," << bottom_left_y << "," << top_right_x << "," << top_right_y;

  std::string wms_url = base_url + "service=WMS&version=1.3.0&request=GetMap&layers=WMS_NI_DOP20&styles=&" + "crs=EPSG:25832&"
                      + "bbox=" + bbox_stream.str() + "&width=" + std::to_string( image_resolution )
                      + "&height=" + std::to_string( image_resolution ) + "&format=image/jpeg";

  // Construct the wget command
  std::string wget_command = "wget -O " + map_storage_path + "/" + std::to_string( map_tile_x ) + ":" + std::to_string( map_tile_y )
                           + ".jpg \"" + wms_url + "\"";

  // Execute the wget command to download the image
  int download_status = std::system( wget_command.c_str() );

  if( download_status != 0 )
  {
    std::cerr << "ERROR in visualizer: Failed to download map image." << std::endl;
    return std::nullopt;
  }

  // Load the downloaded image
  map_image = cv::imread( map_storage_path + "/" + std::to_string( map_tile_x ) + ":" + std::to_string( map_tile_y ) + ".jpg" );

  if( map_image.empty() )
  {
    std::cerr << "ERROR in visualizer: Downloaded map image is invalid." << std::endl;
    return std::nullopt;
  }

  return map_image;
}

// Main function that converts a map image to an occupancy grid
nav_msgs::msg::OccupancyGrid
generate_occupancy_grid( const Offset &offset, const dynamics::VehicleStateDynamic &vehicle_state, const std::string &map_storage_path,
                         bool networking_disabled )
{

  double tile_size = 50; // generate new image if the car leaves this range
  double map_size  = 50; // show is size

  double pixels_per_meter = 5;
  int    image_pixels     = ( tile_size + map_size ) * pixels_per_meter;

  // Prepare the occupancy grid message
  nav_msgs::msg::OccupancyGrid occupancy_grid_msg;

  // Identify which map tile corresponds to the vehicle's current position
  int map_tile_x = std::floor( vehicle_state.x / tile_size );
  int map_tile_y = std::floor( vehicle_state.y / tile_size );

  // Load or download the map image for the current tile
  std::optional<cv::Mat> map_image = fetch_map_image( map_tile_x, map_tile_y, tile_size, map_size, image_pixels, map_storage_path,
                                                      networking_disabled );

  if( !map_image )
  {
    return occupancy_grid_msg;
  }

  // Convert the map image to grayscale for easier processing
  cv::Mat grayscale_image;
  cv::cvtColor( map_image.value(), grayscale_image, cv::COLOR_BGR2GRAY );

  occupancy_grid_msg.header.frame_id = "visualization_offset";
  occupancy_grid_msg.info.resolution = 1.0 / pixels_per_meter;

  occupancy_grid_msg.info.width  = grayscale_image.cols;
  occupancy_grid_msg.info.height = grayscale_image.rows;

  double map_origin_x = map_tile_x * tile_size - offset.x - map_size / 2;
  double map_origin_y = map_tile_y * tile_size - offset.y + map_size / 2 + tile_size;

  occupancy_grid_msg.info.origin.position.x = map_origin_x;
  occupancy_grid_msg.info.origin.position.y = map_origin_y;
  occupancy_grid_msg.info.origin.position.z = -0.8;

  // Set the orientation of the map (rotate 180 degrees to match the desired frame)
  tf2::Quaternion orientation;
  orientation.setRPY( 3.14, 0, 0 );
  occupancy_grid_msg.info.origin.orientation.x = orientation.x();
  occupancy_grid_msg.info.origin.orientation.y = orientation.y();
  occupancy_grid_msg.info.origin.orientation.z = orientation.z();
  occupancy_grid_msg.info.origin.orientation.w = orientation.w();

  // Populate the occupancy grid data (scale grayscale 0-255 to occupancy 0-100)
  occupancy_grid_msg.data.reserve( occupancy_grid_msg.info.width * occupancy_grid_msg.info.height );

  for( int row = 0; row < grayscale_image.rows; ++row )
  {
    for( int col = 0; col < grayscale_image.cols; ++col )
    {
      int occupancy_value = ( grayscale_image.at<uchar>( row, col ) * 100 ) / 255;
      occupancy_grid_msg.data.push_back( 100 - occupancy_value );
    }
  }

  return occupancy_grid_msg;
}

sensor_msgs::msg::PointCloud2
generate_pointcloud2( const Offset &offset, const dynamics::VehicleStateDynamic &vehicle_state, const std::string &map_storage_path,
                      bool networking_disabled )
{
  double tile_size        = 100; // Generate new image if the car leaves this range
  double map_size         = 100; // Visible size
  double pixels_per_meter = 5;
  int    image_pixels     = ( tile_size + map_size ) * pixels_per_meter;

  // Prepare the PointCloud2 message
  sensor_msgs::msg::PointCloud2 cloud_msg;

  // Identify which map tile corresponds to the vehicle's current position
  int map_tile_x = std::floor( vehicle_state.x / tile_size );
  int map_tile_y = std::floor( vehicle_state.y / tile_size );

  // Load the map image for the current tile
  std::optional<cv::Mat> map_image = fetch_map_image( map_tile_x, map_tile_y, tile_size, map_size, image_pixels, map_storage_path,
                                                      networking_disabled );


  cloud_msg.header.frame_id = "visualization_offset";
  cloud_msg.header.stamp    = rclcpp::Clock().now();

  if( !map_image )
  {
    return cloud_msg; // Return empty if no image is available
  }
  cloud_msg.width    = map_image.value().cols;
  cloud_msg.height   = map_image.value().rows;
  cloud_msg.is_dense = true;


  // Define fields: x, y, z (position) and rgb (color)
  sensor_msgs::PointCloud2Modifier modifier( cloud_msg );
  modifier.setPointCloud2Fields( 4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                 sensor_msgs::msg::PointField::FLOAT32, "rgb", 1, sensor_msgs::msg::PointField::FLOAT32 );

  // Set the origin of the map
  double map_origin_x = map_tile_x * tile_size - offset.x - map_size / 2;
  double map_origin_y = map_tile_y * tile_size - offset.y + map_size / 2 + tile_size;

  sensor_msgs::PointCloud2Iterator<float>   iter_x( cloud_msg, "x" );
  sensor_msgs::PointCloud2Iterator<float>   iter_y( cloud_msg, "y" );
  sensor_msgs::PointCloud2Iterator<float>   iter_z( cloud_msg, "z" );
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r( cloud_msg, "rgb" );

  for( int row = 0; row < map_image.value().rows; ++row )
  {
    for( int col = 0; col < map_image.value().cols; ++col )
    {
      cv::Vec3b color = map_image.value().at<cv::Vec3b>( row, col );
      uchar     b     = color[0];
      uchar     g     = color[1];
      uchar     r     = color[2];

      // Convert image pixel to real-world coordinates
      *iter_x = map_origin_x + col / pixels_per_meter;
      *iter_y = map_origin_y - row / pixels_per_meter;
      *iter_z = -0.8; // Flat ground

      // Encode RGB color in a single 32-bit field
      uint32_t rgb = ( static_cast<uint32_t>( r ) << 16 ) | ( static_cast<uint32_t>( g ) << 8 ) | static_cast<uint32_t>( b );
      *reinterpret_cast<uint32_t *>( &( *iter_r ) ) = rgb;

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_r;
    }
  }

  return cloud_msg;
}

} // namespace map_image
} // namespace visualizer
} // namespace adore
