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
#include "adore_dynamics_conversions.hpp"
#include "adore_map_conversions.hpp"
#include "adore_math/angles.h"
#include "adore_ros2_msgs/msg/traffic_prediction.hpp"
#include "adore_ros2_msgs/msg/vehicle_state_dynamic.hpp"
#include <adore_ros2_msgs/msg/traffic_participant_set.hpp>
#include <adore_ros2_msgs/msg/trajectory.hpp>

#include "map_image_visualization.hpp"
#include "state_buffer.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualizer_conversions.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace adore
{
namespace visualizer
{

class Visualizer : public rclcpp::Node
{
private:

  rclcpp::TimerBase::SharedPtr main_timer;

  std::unique_ptr<tf2_ros::TransformBroadcaster> visualisation_transform_broadcaster;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr                                              map_cloud_publisher;
  std::unordered_map<std::string, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr>      marker_publishers;
  std::unordered_map<std::string, rclcpp::Publisher<adore_ros2_msgs::msg::TrajectoryTranspose>::SharedPtr> trajectory_publishers;

  rclcpp::Subscription<adore_ros2_msgs::msg::VehicleStateDynamic>::SharedPtr                         subscriber_vehicle_state_dynamic;
  std::unordered_map<std::string, rclcpp::Subscription<adore_ros2_msgs::msg::Trajectory>::SharedPtr> trajectory_subscriptions;
  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr>                               dynamic_subscriptions;
  std::unordered_map<std::string, MarkerArray>                                                       markers_to_publish;

  template<typename MsgT>
  void update_dynamic_subscriptions( const std::string& expected_type );
  void update_trajectory_subscriptions();

  // callback functions
  void vehicle_state_dynamic_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg );
  void trajectory_callback( const adore_ros2_msgs::msg::Trajectory::SharedPtr msg, const std::string& trajectory_type );
  void timer_callback();

  StateBuffer                                  state_buffer;
  std::optional<dynamics::VehicleStateDynamic> first_state;
  std::optional<dynamics::VehicleStateDynamic> latest_state;
  Offset                                       offset;

  std::string maps_folder;


  // Function to create and store subscribers for various topics
  void create_subscribers();

  // Function to create publishers for various topics
  void create_publishers();
  void update_all_dynamic_subscriptions();

  void publish_visualization_offset();


public:

  Visualizer();
};

template<typename MsgT>
void
Visualizer::update_dynamic_subscriptions( const std::string& expected_type )
{
  // get_topic_names_and_types() returns a map<string, vector<string>> of all topics and their types.
  auto topic_names_and_types = get_topic_names_and_types();

  for( const auto& topic_pair : topic_names_and_types )
  {
    const std::string&              topic_name = topic_pair.first;
    const std::vector<std::string>& types      = topic_pair.second;

    // Check whether this topic advertises our expected type.
    if( std::find( types.begin(), types.end(), expected_type ) != types.end() )
    {
      // If we haven't already subscribed to this topic, do so.
      if( dynamic_subscriptions.find( topic_name ) == dynamic_subscriptions.end() )
      {
        marker_publishers[topic_name]     = create_publisher<visualization_msgs::msg::MarkerArray>( "visualize_" + topic_name, 10 );
        markers_to_publish[topic_name]    = MarkerArray();
        auto subscription                 = create_subscription<MsgT>( topic_name, 1, [this, topic_name]( const MsgT& msg ) {
          // Use the standard conversion function to generate a MarkerArray.
          markers_to_publish[topic_name] = conversions::to_marker_array( msg, offset );
        } );
        dynamic_subscriptions[topic_name] = subscription;
        RCLCPP_INFO( get_logger(), "Dynamically subscribed to topic: %s", topic_name.c_str() );
      }
    }
  }
}
} // namespace visualizer
} // namespace adore
