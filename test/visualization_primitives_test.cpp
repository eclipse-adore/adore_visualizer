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

#include <cmath>
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "color_palette.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using adore::visualizer::Color;
namespace colors     = adore::visualizer::colors;
namespace primitives = adore::visualizer::primitives;

TEST( VisualizationPrimitives, rectangle_marker_basics )
{
  const double x       = 1.0;
  const double y       = 2.0;
  const double z       = 0.5;
  const double length  = 4.0;
  const double width   = 1.5;
  const double height  = 0.8;
  const double heading = M_PI / 4.0;

  const std::string ns = "test_rectangle";
  const int         id = 42;
  const Color       color{ colors::soft_blue };

  auto marker = primitives::create_rectangle_marker( x, y, z, length, width, height, heading, ns, id, color );

  // Namespace / ID
  EXPECT_EQ( marker.ns, ns );
  EXPECT_EQ( marker.id, id );

  // Position
  EXPECT_DOUBLE_EQ( marker.pose.position.x, x );
  EXPECT_DOUBLE_EQ( marker.pose.position.y, y );
  EXPECT_DOUBLE_EQ( marker.pose.position.z, z );

  // Scale
  EXPECT_DOUBLE_EQ( marker.scale.x, length );
  EXPECT_DOUBLE_EQ( marker.scale.y, width );
  EXPECT_DOUBLE_EQ( marker.scale.z, height );

  // Type
  EXPECT_EQ( marker.type, decltype( marker )::CUBE );

  // Color
  EXPECT_FLOAT_EQ( marker.color.r, color[0] );
  EXPECT_FLOAT_EQ( marker.color.g, color[1] );
  EXPECT_FLOAT_EQ( marker.color.b, color[2] );
  EXPECT_FLOAT_EQ( marker.color.a, color[3] );

  // Orientation: yaw-only quaternion
  const double expected_z = std::sin( heading * 0.5 );
  const double expected_w = std::cos( heading * 0.5 );

  EXPECT_NEAR( marker.pose.orientation.x, 0.0, 1e-9 );
  EXPECT_NEAR( marker.pose.orientation.y, 0.0, 1e-9 );
  EXPECT_NEAR( marker.pose.orientation.z, expected_z, 1e-9 );
  EXPECT_NEAR( marker.pose.orientation.w, expected_w, 1e-9 );
}

TEST( VisualizationPrimitives, sphere_marker_basics )
{
  const double x     = -3.0;
  const double y     = 0.5;
  const double z     = 1.2;
  const double scale = 0.7;

  const std::string ns = "test_sphere";
  const int         id = 3;
  const Color       color{ colors::soft_orange };

  auto marker = primitives::create_sphere_marker( x, y, z, scale, ns, id, color );

  EXPECT_EQ( marker.ns, ns );
  EXPECT_EQ( marker.id, id );
  EXPECT_EQ( marker.type, decltype( marker )::SPHERE );

  EXPECT_DOUBLE_EQ( marker.pose.position.x, x );
  EXPECT_DOUBLE_EQ( marker.pose.position.y, y );
  EXPECT_DOUBLE_EQ( marker.pose.position.z, z );

  EXPECT_DOUBLE_EQ( marker.scale.x, scale );
  EXPECT_DOUBLE_EQ( marker.scale.y, scale );
  EXPECT_DOUBLE_EQ( marker.scale.z, scale );

  EXPECT_FLOAT_EQ( marker.color.r, color[0] );
  EXPECT_FLOAT_EQ( marker.color.g, color[1] );
  EXPECT_FLOAT_EQ( marker.color.b, color[2] );
  EXPECT_FLOAT_EQ( marker.color.a, color[3] );
}

TEST( VisualizationPrimitives, line_marker_from_geometry_points )
{
  geometry_msgs::msg::Point start;
  start.x = 0.0;
  start.y = 0.0;
  start.z = 0.0;

  geometry_msgs::msg::Point end;
  end.x = 5.0;
  end.y = -1.0;
  end.z = 0.0;

  const std::string ns = "test_line";
  const int         id = 7;
  const double      width{ 0.15 };
  const Color       color{ colors::red };

  auto marker = primitives::create_line_marker( start, end, ns, id, width, color );

  EXPECT_EQ( marker.ns, ns );
  EXPECT_EQ( marker.id, id );
  EXPECT_EQ( marker.type, decltype( marker )::LINE_STRIP );

  ASSERT_EQ( marker.points.size(), 2u );
  EXPECT_DOUBLE_EQ( marker.points[0].x, start.x );
  EXPECT_DOUBLE_EQ( marker.points[0].y, start.y );
  EXPECT_DOUBLE_EQ( marker.points[0].z, 0.0 );

  EXPECT_DOUBLE_EQ( marker.points[1].x, end.x );
  EXPECT_DOUBLE_EQ( marker.points[1].y, end.y );
  EXPECT_DOUBLE_EQ( marker.points[1].z, 0.0 );

  EXPECT_DOUBLE_EQ( marker.scale.x, width );

  EXPECT_FLOAT_EQ( marker.color.r, color[0] );
  EXPECT_FLOAT_EQ( marker.color.g, color[1] );
  EXPECT_FLOAT_EQ( marker.color.b, color[2] );
  EXPECT_FLOAT_EQ( marker.color.a, color[3] );
}

TEST( VisualizationPrimitives, flat_line_marker_builds_triangles )
{
  std::vector<geometry_msgs::msg::Point> points( 3 );
  points[0].x = 0.0;
  points[0].y = 0.0;

  points[1].x = 1.0;
  points[1].y = 0.0;

  points[2].x = 1.0;
  points[2].y = 1.0;

  const std::string ns = "flat_line";
  const int         id = 1;
  const double      width{ 0.4 };
  const Color       color{ colors::soft_green };

  auto marker = primitives::create_flat_line_marker( points, ns, id, width, color );

  EXPECT_EQ( marker.ns, ns );
  EXPECT_EQ( marker.id, id );
  EXPECT_EQ( marker.type, decltype( marker )::TRIANGLE_LIST );

  EXPECT_FLOAT_EQ( marker.color.r, color[0] );
  EXPECT_FLOAT_EQ( marker.color.g, color[1] );
  EXPECT_FLOAT_EQ( marker.color.b, color[2] );
  EXPECT_FLOAT_EQ( marker.color.a, color[3] );

  // For N points we get (N-1) segments, each as 2 triangles = 6 vertices.
  const std::size_t expected_vertices = 6 * ( points.size() - 1 );
  EXPECT_EQ( marker.points.size(), expected_vertices );
}

TEST( VisualizationPrimitives, flat_line_marker_too_few_points_returns_default )
{
  std::vector<geometry_msgs::msg::Point> points( 2 ); // < 3

  auto marker = primitives::create_flat_line_marker( points, "ns_ignored", 0, 1.0, colors::soft_red );

  // Early return should give a default-constructed marker.
  EXPECT_TRUE( marker.ns.empty() );
  EXPECT_TRUE( marker.points.empty() );
  EXPECT_EQ( marker.type, 0 ); // default enum value
}

TEST( VisualizationPrimitives, transform_marker_translates_line_points )
{
  geometry_msgs::msg::Point start;
  start.x = 1.0;
  start.y = 2.0;
  start.z = 0.0;

  geometry_msgs::msg::Point end;
  end.x = -1.0;
  end.y = 3.0;
  end.z = 0.0;

  auto marker = primitives::create_line_marker( start, end, "line_to_transform", 10, 0.1, colors::orange );

  // Set the source frame on the marker header.
  marker.header.frame_id = "world";

  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id         = "world";
  tf.child_frame_id          = "offset";
  tf.transform.translation.x = 5.0;
  tf.transform.translation.y = -2.0;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.x    = 0.0;
  tf.transform.rotation.y    = 0.0;
  tf.transform.rotation.z    = 0.0;
  tf.transform.rotation.w    = 1.0; // Identity rotation

  const auto before_p0 = marker.points[0];
  const auto before_p1 = marker.points[1];

  primitives::transform_marker( marker, tf );

  EXPECT_NEAR( marker.points[0].x, before_p0.x + tf.transform.translation.x, 1e-9 );
  EXPECT_NEAR( marker.points[0].y, before_p0.y + tf.transform.translation.y, 1e-9 );
  EXPECT_NEAR( marker.points[0].z, before_p0.z + tf.transform.translation.z, 1e-9 );

  EXPECT_NEAR( marker.points[1].x, before_p1.x + tf.transform.translation.x, 1e-9 );
  EXPECT_NEAR( marker.points[1].y, before_p1.y + tf.transform.translation.y, 1e-9 );
  EXPECT_NEAR( marker.points[1].z, before_p1.z + tf.transform.translation.z, 1e-9 );
}
