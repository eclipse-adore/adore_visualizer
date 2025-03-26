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
#include "infrastructure_visualizer.hpp"

using namespace std::chrono_literals;

namespace adore
{
namespace visualizer
{

InfrastructureVisualizer::InfrastructureVisualizer() : Node( "infrastructure_visualizer_node" )
{
}

} // namespace visualizer
} // namespace adore

int
main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );

  std::shared_ptr<adore::visualizer::InfrastructureVisualizer> node = std::make_shared<adore::visualizer::InfrastructureVisualizer>();
  rclcpp::spin( node );
  rclcpp::shutdown();
}
