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
#include "building_visualization.hpp"
#include <cmath>
#include <adore_map/lat_long_conversions.hpp>
#include <adore_math/distance.h>
#include <adore_math/point.h>

 
namespace adore
{
namespace visualizer
{
namespace buildings
{

  std::vector<Building> extract_buildings_from_road_features(const std::string &assets_folder)
  {
    std::vector<Building> buildings_in_file;

    // This right now goes to the assets/map/road_features, and it should be elsewhere, but is fine for now
    std::string road_features_file_path = assets_folder + "/road_features/road_features.json";

    std::ifstream features_file(road_features_file_path);

    if ( !features_file.is_open() ) // If it was unable to read a file
    {
      std::cerr << "Unable to load road features, file or path is invalid (" << road_features_file_path << ")" << std::endl;
      return buildings_in_file;
    }

    json j = json::parse(features_file);

    for ( const auto& [key, item] : j.items() )
    {
      if ( !item.is_object() )
        continue;

      // This should later get added back to make sure its a building
      // if ( !item.contains("type") )
      //   continue;

      if ( !item.contains("centroid") || !item.contains("area_m2") || !item.contains("width") || !item.contains("length") )
        continue;

      const auto& building_position_utm = item["centroid"];
      const auto& building_size_m2 = item["area_m2"];
      const auto& building_width = item["width"];
      const auto& building_length = item["length"];
      const auto& building_orientation = item["orientation"];

      Building new_building = Building { building_position_utm[0], building_position_utm[1], building_size_m2, building_width, building_length, building_orientation, "UTM32U" };

      buildings_in_file.push_back( new_building );
    }

    return buildings_in_file;
  };

  std::vector<int> get_nearby_buildings(const std::vector<Building>& all_buildings, const double& current_position_utm_x, const double& current_position_utm_y, const double& show_distance)
  {
    std::vector<int> building_ids_inside_of_show_area; 
    
    for ( int i = 0; i < all_buildings.size(); i++ )
    {
      math::Point2d building_position = math::Point2d { all_buildings[i].position_utm_x, all_buildings[i].position_utm_y };
      math::Point2d vehicle_position = math::Point2d { current_position_utm_x, current_position_utm_y };

      if ( math::distance_2d(building_position, vehicle_position) < show_distance )
      {
        building_ids_inside_of_show_area.push_back( i );
      }
    }

    return building_ids_inside_of_show_area;
  }
} // namespace buildings
} // namespace visualizer
} // namespace adore
