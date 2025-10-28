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
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "adore_map/lat_long_conversions.hpp"

#include "adore_math/distance.h"
#include "adore_math/polygon.h"

using json = nlohmann::json; 

namespace adore
{
namespace visualizer
{
namespace buildings
{
  struct Building
  {
    double position_utm_x;
    double position_utm_y;
    double area_m2;
    double width;
    double length;
    double orientation;
    std::string utm_zone;
  };
  
  std::vector<Building> extract_buildings_from_road_features(const std::string &features_storage_path);

  // This gets the keys of all buildings
  std::vector<int> get_nearby_buildings(const std::vector<Building>& all_buildings, const double& current_position_utm_x, const double& current_position_utm_y, const double& show_distance);

} // namespace buildings
} // namespace visualizer
} // namespace adore
