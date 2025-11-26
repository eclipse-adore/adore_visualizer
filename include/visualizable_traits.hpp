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
#include <type_traits>
#include <utility>

#include "visualizer_conversions.hpp"

namespace adore
{
namespace visualizer
{

// --- MarkerArray conversion trait ------------------------------------------

template<typename MsgT, typename = void>
struct marker_array_conversion
{
  static constexpr bool supported = false;
};

template<typename MsgT>
struct marker_array_conversion<MsgT, std::void_t<decltype( conversions::to_marker_array( std::declval<const MsgT&>() ) )>>
{
  static constexpr bool supported = true;

  using result_type = std::decay_t<decltype( conversions::to_marker_array( std::declval<const MsgT&>() ) )>;

  static result_type
  convert( const MsgT& msg )
  {
    return conversions::to_marker_array( msg );
  }
};

template<typename MsgT>
inline constexpr bool has_marker_array_conversion = marker_array_conversion<MsgT>::supported;

// --- TrajectoryTranspose conversion trait ----------------------------------

template<typename MsgT, typename = void>
struct trajectory_transpose_conversion
{
  static constexpr bool supported = false;
};

template<typename MsgT>
struct trajectory_transpose_conversion<MsgT, std::void_t<decltype( dynamics::conversions::transpose( std::declval<const MsgT&>() ) )>>
{
  static constexpr bool supported = true;

  using result_type = std::decay_t<decltype( dynamics::conversions::transpose( std::declval<const MsgT&>() ) )>;

  static result_type
  convert( const MsgT& msg )
  {
    return dynamics::conversions::transpose( msg );
  }
};

template<typename MsgT>
inline constexpr bool has_trajectory_transpose_conversion = trajectory_transpose_conversion<MsgT>::supported;

// ---------------- NavSatFix conversion trait --------------------------------

template<typename MsgT, typename = void>
struct nav_sat_fix_conversion
{
  static constexpr bool supported = false;
};

template<typename MsgT>
struct nav_sat_fix_conversion<MsgT, std::void_t<decltype( conversions::to_nav_sat_fix( std::declval<const MsgT&>() ) )>>
{
  static constexpr bool supported = true;

  using result_type = std::decay_t<decltype( conversions::to_nav_sat_fix( std::declval<const MsgT&>() ) )>;

  static result_type
  convert( const MsgT& msg )
  {
    return conversions::to_nav_sat_fix( msg );
  }
};

template<typename MsgT>
inline constexpr bool has_nav_sat_fix_conversion = nav_sat_fix_conversion<MsgT>::supported;

// ---------------- GeoJSON conversion trait ----------------------------------.

template<typename MsgT, typename = void>
struct geo_json_conversion
{
  static constexpr bool supported = false;
};

template<typename MsgT>
struct geo_json_conversion<MsgT, std::void_t<decltype( conversions::to_geo_json( std::declval<const MsgT&>() ) )>>
{
  static constexpr bool supported = true;

  using result_type = std::decay_t<decltype( conversions::to_geo_json( std::declval<const MsgT&>() ) )>;

  static result_type
  convert( const MsgT& msg )
  {
    return conversions::to_geo_json( msg );
  }
};

template<typename MsgT>
inline constexpr bool has_geo_json_conversion = geo_json_conversion<MsgT>::supported;

} // namespace visualizer
} // namespace adore
