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
 *    Marko Mizdrak
 *    Mikkel Skov Maarssø
 ********************************************************************************/
#pragma once
#include "adore_dynamics_conversions.hpp"
#include "adore_map_conversions.hpp"
#include "adore_math/angles.h"
#include "adore_ros2_msgs/msg/traffic_prediction.hpp"
#include "adore_ros2_msgs/msg/vehicle_state_dynamic.hpp"
#include <adore_ros2_msgs/msg/traffic_participant_set.hpp>
#include <adore_ros2_msgs/msg/trajectory.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace adore
{
namespace visualizer
{


class InfrastructureVisualizer: public rclcpp::Node
{
private:


  // Subscriber/Publisher Creation Functions
  // void create_subscribers();
  // void create_publishers();

  // void timer_callback();

public:

  InfrastructureVisualizer();
};

} // namespace visualizer
} // namespace adore
