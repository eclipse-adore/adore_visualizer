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

#include <cmath>

#include <iomanip>
#include <optional>
#include <sstream>
#include <system_error>

namespace adore
{
namespace visualizer
{
namespace map_image
{


// Construct the file path for the map image using a given extension
std::string
build_image_file_path( int map_tile_x, int map_tile_y, const std::string &map_storage_path, const std::string &extension )
{
  return map_storage_path + "/" + std::to_string( map_tile_x ) + ":" + std::to_string( map_tile_y ) + extension;
}

// Build a wget command for downloading an image
std::string
build_wget_command( const std::string &file_path, const std::string &url )
{
  std::ostringstream oss;
  oss << "wget -O " << file_path << " ";
  oss << "\"" << url << "\"";
  return oss.str();
}

// Compute the bounding box string for a given map tile.
// For the closed API, use integer coordinates; for the open API, use fixed precision doubles.
std::string
compute_bbox_string( int map_tile_x, int map_tile_y, double tile_size, double map_size )
{
  double             bottom_left_x = map_tile_x * tile_size - map_size / 2.0;
  double             bottom_left_y = map_tile_y * tile_size - map_size / 2.0;
  double             top_right_x   = map_tile_x * tile_size + tile_size + map_size / 2.0;
  double             top_right_y   = map_tile_y * tile_size + tile_size + map_size / 2.0;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision( 6 ) << bottom_left_x << "," << bottom_left_y << "," << top_right_x << "," << top_right_y;
  return oss.str();
}

// Compute the origin of the map in world coordinates.
std::pair<double, double>
compute_map_origin( int map_tile_x, int map_tile_y, double tile_size, double map_size, const Offset &offset )
{
  double origin_x = map_tile_x * tile_size - offset.x - map_size / 2;
  double origin_y = map_tile_y * tile_size - offset.y + map_size / 2 + tile_size;
  return { origin_x, origin_y };
}

// Fetch the map image from local storage or via network if needed
std::optional<cv::Mat>
fetch_map_image( int map_tile_x, int map_tile_y, double tile_size, double map_size, int image_resolution,
                 const std::string &map_storage_path, bool networking_disabled, const std::string &api_key )
{
  // Try loading the image from the local file system first.
  auto map_image = fetch_map_image_local( map_tile_x, map_tile_y, tile_size, map_size, image_resolution, map_storage_path );
  if( map_image || networking_disabled )
  {
    return map_image;
  }

  // If an API key is provided, use the closed API; otherwise, use the open API.
  if( !api_key.empty() )
  {
    return fetch_map_image_closed( map_tile_x, map_tile_y, tile_size, map_size, image_resolution, map_storage_path, api_key );
  }
  return fetch_map_image_open( map_tile_x, map_tile_y, tile_size, map_size, image_resolution, map_storage_path );
}

// Load map image from the local file system (tries PNG then JPG)
std::optional<cv::Mat>
fetch_map_image_local( int map_tile_x, int map_tile_y, double tile_size, double map_size, int image_resolution,
                       const std::string &map_storage_path )
{
  // Construct file paths for PNG and JPG formats.
  std::string png_path = build_image_file_path( map_tile_x, map_tile_y, map_storage_path, ".png" );
  std::string jpg_path = build_image_file_path( map_tile_x, map_tile_y, map_storage_path, ".jpg" );

  cv::Mat map_image = cv::imread( png_path );
  if( !map_image.empty() )
  {
    return map_image;
  }
  map_image = cv::imread( jpg_path );
  if( !map_image.empty() )
  {
    return map_image;
  }

  std::cerr << "No local image found" << std::endl;
  return std::nullopt;
}

// Download map image using the closed API (requires an API key)
std::optional<cv::Mat>
fetch_map_image_closed( int map_tile_x, int map_tile_y, double tile_size, double map_size, int image_resolution,
                        const std::string &map_storage_path, const std::string &api_key )
{
  std::cerr << "Getting closed API image" << std::endl;

  // Build the base URL for the closed API request.
  std::ostringstream url_stream;
  url_stream << "https://adore-ts_reader:" << api_key
             << "@ts.dlr.de/mumme-server/bs-gov/wms?service=WMS&version=1.1.0&request=GetMap&layers=bs-gov:dop-2020"
                "&styles=&width="
             << image_resolution << "&height=" << image_resolution << "&srs=EPSG:32632&format=image/jpeg&bbox=";
  const std::string base_url = url_stream.str();

  // Compute the bounding box using integer coordinates.
  std::string       bbox_str  = compute_bbox_string( map_tile_x, map_tile_y, tile_size, map_size );
  const std::string file_path = build_image_file_path( map_tile_x, map_tile_y, map_storage_path, ".jpg" );

  // Build and execute the wget command.
  std::string wget_command = build_wget_command( file_path, base_url + bbox_str );
  if( std::system( wget_command.c_str() ) != 0 )
  {
    return std::nullopt;
  }

  // Load the downloaded image.
  cv::Mat map_image = cv::imread( file_path );
  if( map_image.empty() )
  {
    std::cerr << "ERROR in visualizer: Download of map provided wrong result" << std::endl;
    return std::nullopt;
  }
  return map_image;
}

// Download map image using the open API (no API key required)
std::optional<cv::Mat>
fetch_map_image_open( int map_tile_x, int map_tile_y, double tile_size, double map_size, int image_resolution,
                      const std::string &map_storage_path )
{
  // Base URL for the open WMS service.
  const std::string base_url = "https://opendata.lgln.niedersachsen.de/doorman/noauth/dop_wms?";

  // Compute the bounding box using double precision.
  std::string bbox_str = compute_bbox_string( map_tile_x, map_tile_y, tile_size, map_size );

  // Build the complete WMS URL.
  std::ostringstream url_stream;
  url_stream << base_url << "service=WMS&version=1.3.0&request=GetMap&layers=WMS_NI_DOP20&styles=&crs=EPSG:25832&"
             << "bbox=" << bbox_str << "&width=" << image_resolution << "&height=" << image_resolution << "&format=image/jpeg";
  const std::string wms_url = url_stream.str();

  const std::string file_path = build_image_file_path( map_tile_x, map_tile_y, map_storage_path, ".jpg" );

  // Build and execute the wget command with quoted URL.
  std::string wget_command = build_wget_command( file_path, wms_url );
  if( std::system( wget_command.c_str() ) != 0 )
  {
    std::cerr << "ERROR in visualizer: Failed to download map image." << std::endl;
    return std::nullopt;
  }

  // Load the downloaded image.
  cv::Mat map_image = cv::imread( file_path );
  if( map_image.empty() )
  {
    return std::nullopt;
  }
  return map_image;
}

// Convert a map image to an occupancy grid
nav_msgs::msg::OccupancyGrid
generate_occupancy_grid( const Offset &offset, const dynamics::VehicleStateDynamic &vehicle_state, const std::string &map_storage_path,
                         bool networking_disabled, const std::string &api_key )
{
  // Configuration parameters.
  const double tile_size        = 50; // New image generated if vehicle leaves this range.
  const double map_size         = 50; // Visible map size.
  const double pixels_per_meter = 5;
  const int    image_pixels     = static_cast<int>( ( tile_size + map_size ) * pixels_per_meter );

  nav_msgs::msg::OccupancyGrid occupancy_grid_msg;

  // Determine the current map tile based on vehicle position.
  const int map_tile_x = std::floor( vehicle_state.x / tile_size );
  const int map_tile_y = std::floor( vehicle_state.y / tile_size );

  // Fetch or download the map image for the current tile.
  auto map_image = fetch_map_image( map_tile_x, map_tile_y, tile_size, map_size, image_pixels, map_storage_path, networking_disabled,
                                    api_key );
  if( !map_image )
  {
    return occupancy_grid_msg;
  }

  // Convert the image to grayscale for processing.
  cv::Mat grayscale_image;
  cv::cvtColor( map_image.value(), grayscale_image, cv::COLOR_BGR2GRAY );

  // Set up occupancy grid metadata.
  occupancy_grid_msg.header.frame_id = "visualization_offset";
  occupancy_grid_msg.info.resolution = 1.0 / pixels_per_meter;
  occupancy_grid_msg.info.width      = grayscale_image.cols;
  occupancy_grid_msg.info.height     = grayscale_image.rows;

  // Compute the map origin.
  auto [map_origin_x, map_origin_y]         = compute_map_origin( map_tile_x, map_tile_y, tile_size, map_size, offset );
  occupancy_grid_msg.info.origin.position.x = map_origin_x;
  occupancy_grid_msg.info.origin.position.y = map_origin_y;
  occupancy_grid_msg.info.origin.position.z = -0.8;

  // Rotate the map 180 degrees.
  tf2::Quaternion orientation;
  orientation.setRPY( 3.14, 0, 0 );
  occupancy_grid_msg.info.origin.orientation.x = orientation.x();
  occupancy_grid_msg.info.origin.orientation.y = orientation.y();
  occupancy_grid_msg.info.origin.orientation.z = orientation.z();
  occupancy_grid_msg.info.origin.orientation.w = orientation.w();

  // Populate the occupancy grid by scaling grayscale values (0-255) to occupancy (0-100).
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

// Generate a PointCloud2 message from a map image and cache it by tile index
std::pair<TileKey, sensor_msgs::msg::PointCloud2>
generate_pointcloud2( const Offset &offset, const dynamics::VehicleStateDynamic &vehicle_state, const std::string &map_storage_path,
                      bool networking_disabled, TileCache &tile_cache, const std::string &api_key, bool grayscale )
{
  // Configuration parameters.
  const double magenta_correction_factor = 0.0;
  const double tile_size                 = 100; // New image generated if vehicle leaves this range.
  const double map_size                  = 100; // Visible map size.
  const double pixels_per_meter          = 5;
  const int    image_pixels              = static_cast<int>( ( tile_size + map_size ) * pixels_per_meter );

  // Determine the current map tile.
  const int map_tile_x = std::floor( vehicle_state.x / tile_size );
  const int map_tile_y = std::floor( vehicle_state.y / tile_size );
  TileKey   tile_index = { map_tile_x, map_tile_y };

  // Return cached PointCloud2 if available.
  if( tile_cache.find( tile_index ) != tile_cache.end() )
  {
    return { tile_index, tile_cache[tile_index] };
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = "visualization_offset";
  cloud_msg.header.stamp    = rclcpp::Clock().now();

  // Fetch or download the map image.
  auto map_image = fetch_map_image( map_tile_x, map_tile_y, tile_size, map_size, image_pixels, map_storage_path, networking_disabled,
                                    api_key );
  if( !map_image )
  {
    return { tile_index, cloud_msg };
  }

  cloud_msg.width    = map_image.value().cols;
  cloud_msg.height   = map_image.value().rows;
  cloud_msg.is_dense = true;

  // Define point fields: x, y, z and rgb.
  sensor_msgs::PointCloud2Modifier modifier( cloud_msg );
  modifier.setPointCloud2Fields( 4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                 sensor_msgs::msg::PointField::FLOAT32, "rgb", 1, sensor_msgs::msg::PointField::FLOAT32 );

  // Compute the map origin.
  auto [map_origin_x, map_origin_y] = compute_map_origin( map_tile_x, map_tile_y, tile_size, map_size, offset );

  // Prepare iterators for point data.
  sensor_msgs::PointCloud2Iterator<float>   iter_x( cloud_msg, "x" );
  sensor_msgs::PointCloud2Iterator<float>   iter_y( cloud_msg, "y" );
  sensor_msgs::PointCloud2Iterator<float>   iter_z( cloud_msg, "z" );
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r( cloud_msg, "rgb" );

  // Compute average color for magenta bias correction.
  cv::Scalar mean_color     = cv::mean( map_image.value() );
  double     avg_r          = mean_color[2];
  double     avg_g          = mean_color[1];
  double     avg_b          = mean_color[0];
  double     magenta_offset = ( avg_r + avg_b ) - ( 2 * avg_g );

  // Iterate over each pixel to populate the point cloud.
  for( int row = 0; row < map_image.value().rows; ++row )
  {
    for( int col = 0; col < map_image.value().cols; ++col )
    {
      cv::Vec3b color = map_image.value().at<cv::Vec3b>( row, col );
      uchar     b     = color[0];
      uchar     g     = color[1];
      uchar     r     = color[2];

      if( grayscale )
      {
        b = g = r = static_cast<uchar>( 0.299 * r + 0.587 * g + 0.114 * b );
      }

      // Adjust colors to reduce magenta bias.
      r = static_cast<uchar>( std::clamp( r - magenta_correction_factor * magenta_offset, 0.0, 255.0 ) );
      b = static_cast<uchar>( std::clamp( b - magenta_correction_factor * magenta_offset, 0.0, 255.0 ) );
      g = static_cast<uchar>( std::clamp( g + magenta_correction_factor * magenta_offset, 0.0, 255.0 ) );

      // Set point position.
      *iter_x = map_origin_x + col / pixels_per_meter;
      *iter_y = map_origin_y - row / pixels_per_meter;
      *iter_z = -0.1; // Flat ground.

      // Pack RGB into a single 32-bit integer.
      uint32_t rgb = ( static_cast<uint32_t>( r ) << 16 ) | ( static_cast<uint32_t>( g ) << 8 ) | static_cast<uint32_t>( b );
      *reinterpret_cast<uint32_t *>( &( *iter_r ) ) = rgb;

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_r;
    }
  }

  // Cache the generated PointCloud2 message.
  tile_cache[tile_index] = cloud_msg;
  return { tile_index, cloud_msg };
}

} // namespace map_image
} // namespace visualizer
} // namespace adore
