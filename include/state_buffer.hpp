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
#include <deque>

#include "adore_dynamics_conversions.hpp"

#include <rclcpp/rclcpp.hpp>

namespace adore
{
namespace visualizer
{


// Define a time-based circular buffer for Odometry messages
class StateBuffer
{
public:

  // Constructor that accepts a time window in seconds
  explicit StateBuffer( double time_windowseconds ) :
    time_window( time_windowseconds )
  {}

  // Add an odometry message to the buffer
  void
  add( const adore_ros2_msgs::msg::VehicleStateDynamic& state )
  {
    buffer.emplace_back( state );

    // Prune the buffer to remove old messages outside the time window
    prune_old_messages();
  }

  // Get the odometry buffer
  const std::deque<adore_ros2_msgs::msg::VehicleStateDynamic>&
  get() const
  {
    return buffer;
  }

private:

  void
  prune_old_messages()
  {
    // Use the time from the latest odometry message in the buffer
    double current_time( buffer.back().time );

    // Loop through the buffer and remove old messages outside the time window
    while( !buffer.empty() )
    {
      double oldest_time( buffer.front().time );

      // If the oldest message is within the time window, stop pruning
      if( current_time - oldest_time <= time_window )
      {
        break;
      }

      // Remove the oldest message if it's outside the time window
      buffer.pop_front();
    }
  }

  std::deque<adore_ros2_msgs::msg::VehicleStateDynamic> buffer;      // Stores odometry messages with timestamps
  double                                                time_window; // Maximum time window for keeping messages
};
} // namespace visualizer
} // namespace adore