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

 ********************************************************************************/
#pragma once
#include <cmath>

#include <optional>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "adore_dynamics_conversions.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_primitives.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace adore
{
namespace visualizer
{
namespace map_image
{
std::optional<cv::Mat> fetch_map_image( int map_tile_x, int map_tile_y, double tile_size, double map_size, int image_resolution,
                                        const std::string &map_storage_path, bool networking_disabled );

// Main function that converts a map image to an occupancy grid
nav_msgs::msg::OccupancyGrid generate_occupancy_grid( const Offset &offset, const dynamics::VehicleStateDynamic &vehicle_odometry,
                                                      const std::string &map_storage_path, bool networking_disabled );

sensor_msgs::msg::PointCloud2 generate_pointcloud2( const Offset &offset, const dynamics::VehicleStateDynamic &vehicle_state,
                                                    const std::string &map_storage_path, bool networking_disabled );

} // namespace map_image
} // namespace visualizer
} // namespace adore
