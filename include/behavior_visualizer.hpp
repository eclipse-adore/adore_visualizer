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
#include <cmath>

#include <optional>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "adore_dynamics_conversions.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
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
namespace behavior
{

std::optional<cv::Mat> fetch_traffic_light_state_image(const std::string &images_storage_path, const int traffic_light_state);
std::optional<cv::Mat> fetch_vehicle_state_image(const std::string &asset_storage_path, const float &distance_to_nearest_participant);
sensor_msgs::msg::Image image_to_msg(const cv::Mat& image);

  

} // namespace behavior
} // namespace visualizer
} // namespace adore
