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
#include <cmath>

#include <array>

namespace adore
{
namespace visualizer
{
using Color = std::array<float, 4>;

namespace colors
{
// Define the Tableau 10 color palette with RGBA
constexpr Color white  = { 1.0f, 1.0f, 1.0f, 1.0f };       // #FFFFFF
constexpr Color blue   = { 0.121f, 0.467f, 0.706f, 1.0f }; // #1F77B4
constexpr Color orange = { 1.0f, 0.498f, 0.054f, 1.0f };   // #FF7F0E
constexpr Color green  = { 0.173f, 0.627f, 0.173f, 1.0f }; // #2CA02C
constexpr Color red    = { 0.839f, 0.153f, 0.157f, 1.0f }; // #D62728
constexpr Color purple = { 0.580f, 0.404f, 0.741f, 1.0f }; // #9467BD
constexpr Color brown  = { 0.549f, 0.337f, 0.294f, 1.0f }; // #8C564B
constexpr Color pink   = { 0.867f, 0.518f, 0.604f, 1.0f }; // #E377C2
constexpr Color gray   = { 0.498f, 0.498f, 0.498f, 1.0f }; // #7F7F7F
constexpr Color yellow = { 0.737f, 0.741f, 0.133f, 1.0f }; // #BCBD22
constexpr Color cyan   = { 0.090f, 0.745f, 0.812f, 1.0f }; // #17BECF
constexpr Color black  = { 0.0f, 0.0f, 0.0f, 1.0f };       // #000000

// Define the Tableau 10 color palette with RGBA
constexpr Color soft_blue   = { 0.121f, 0.467f, 0.706f, 0.7f }; // #1F77B4
constexpr Color soft_orange = { 1.0f, 0.498f, 0.054f, 0.7f };   // #FF7F0E
constexpr Color soft_green  = { 0.173f, 0.627f, 0.173f, 0.7f }; // #2CA02C
constexpr Color soft_red    = { 0.839f, 0.153f, 0.157f, 0.7f }; // #D62728
constexpr Color soft_purple = { 0.580f, 0.404f, 0.741f, 0.7f }; // #9467BD
constexpr Color soft_brown  = { 0.549f, 0.337f, 0.294f, 0.7f }; // #8C564B
constexpr Color soft_pink   = { 0.867f, 0.518f, 0.604f, 0.7f }; // #E377C2
constexpr Color soft_gray   = { 0.498f, 0.498f, 0.498f, 0.7f }; // #7F7F7F
constexpr Color soft_yellow = { 0.737f, 0.741f, 0.133f, 0.7f }; // #BCBD22
constexpr Color soft_cyan   = { 0.090f, 0.745f, 0.812f, 0.7f }; // #17BECF
constexpr Color soft_black  = { 0.0f, 0.0f, 0.0f, 0.7f };       // #000000

} // namespace colors

// Function to convert HSV to RGB
inline static Color
hsv_to_rgb( float h, float s, float v )
{
  float r, g, b;

  int   i = static_cast<int>( h * 6 );
  float f = h * 6 - i;
  float p = v * ( 1 - s );
  float q = v * ( 1 - f * s );
  float t = v * ( 1 - ( 1 - f ) * s );

  switch( i % 6 )
  {
    case 0:
      r = v;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = v;
      b = p;
      break;
    case 2:
      r = p;
      g = v;
      b = t;
      break;
    case 3:
      r = p;
      g = q;
      b = v;
      break;
    case 4:
      r = t;
      g = p;
      b = v;
      break;
    case 5:
      r = v;
      g = p;
      b = q;
      break;
  }

  return { r, g, b, 1.0f }; // Alpha is set to 1.0f
}

// Map acceleration (m/s^2) to hue: -3 → 0° (red), +3 → 120° (green).
inline double
accel_to_hue_deg( double acc )
{
  double t = std::clamp( ( acc + 3.0 ) / 6.0, 0.0, 1.0 ); // [-3,3] → [0,1]
  return t / 3.0;
}

} // namespace visualizer
} // namespace adore
