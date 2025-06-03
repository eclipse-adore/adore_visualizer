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
 ********************************************************************************/
#pragma once

#include "adore_map_conversions.hpp"
#include "adore_math/angles.h"
#include "adore_ros2_msgs/msg/goal_point.hpp"
#include "adore_ros2_msgs/msg/route.hpp"
#include "adore_ros2_msgs/msg/trajectory.hpp"
#include <adore_ros2_msgs/msg/caution_zone.hpp>
#include <adore_ros2_msgs/msg/safety_corridor.hpp>
#include <adore_ros2_msgs/msg/traffic_participant_set.hpp>
#include <adore_ros2_msgs/msg/traffic_prediction.hpp>
#include <adore_ros2_msgs/msg/traffic_signals.hpp>
#include <adore_ros2_msgs/msg/visualizable_object.hpp>
#include <adore_ros2_msgs/msg/waypoints.hpp>

#include "color_palette.hpp"
#include "visualization_primitives.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace adore
{
namespace visualizer
{

namespace conversions
{


// Helper functions to convert messages to MarkerArray
MarkerArray to_marker_array( const adore_ros2_msgs::msg::TrafficParticipantSet& msg );

MarkerArray to_marker_array( const adore_ros2_msgs::msg::VehicleStateDynamic& msg );

MarkerArray to_marker_array( const adore_ros2_msgs::msg::Trajectory& set_point_request );

MarkerArray to_marker_array( const adore_ros2_msgs::msg::SafetyCorridor& safety_corridor );

MarkerArray to_marker_array( const adore_ros2_msgs::msg::Map& local_map );

MarkerArray to_marker_array( const adore_ros2_msgs::msg::Route& route );

MarkerArray to_marker_array( const adore_ros2_msgs::msg::GoalPoint& route );

MarkerArray to_marker_array( const adore_ros2_msgs::msg::TrafficSignals& traffic_signals );

MarkerArray to_marker_array( const adore_ros2_msgs::msg::CautionZone& caution_zone );

MarkerArray to_marker_array( const adore_ros2_msgs::msg::Waypoints& waypoints_msg );

MarkerArray to_marker_array( const adore_ros2_msgs::msg::VisualizableObject& msg );


} // namespace conversions
} // namespace visualizer
} // namespace adore
