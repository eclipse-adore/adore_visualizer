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

#include <optional>
#include <string>
#include <vector>

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
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "visualization_primitives.hpp"
#include <foxglove_msgs/msg/geo_json.hpp>
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

using NavSatFix = sensor_msgs::msg::NavSatFix;
using GeoJSON   = foxglove_msgs::msg::GeoJSON;

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

NavSatFix to_nav_sat_fix( const adore_ros2_msgs::msg::VehicleStateDynamic& vehicle_state_dynamic );

GeoJSON to_geo_json( const adore_ros2_msgs::msg::GoalPoint& goal_point );

GeoJSON to_geo_json( const adore_ros2_msgs::msg::Route& route );

std::optional<Marker> get_participant_3d_model( const adore_ros2_msgs::msg::TrafficParticipantDetection& participant );

std::string get_best_fiting_car_3d_model( const adore_ros2_msgs::msg::TrafficParticipantDetection& participant );

} // namespace conversions
} // namespace visualizer
} // namespace adore
