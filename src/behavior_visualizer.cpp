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
 #include "behavior_visualizer.hpp"

namespace adore
{
namespace visualizer
{
namespace behavior
{

std::optional<cv::Mat> fetch_traffic_light_state_image(const std::string &assets_folder, const int traffic_light_state)
{
  std::string image_file_path = "";
  switch (traffic_light_state)
  {
    case 0:
    {
      image_file_path = assets_folder + "/images/traffic_light_red.png";
      break;
    }
    case 1:
    {
      image_file_path = assets_folder + "/images/traffic_light_yellow_to_red.png";
      break;
    }
    case 2:
    {
      image_file_path = assets_folder + "/images/traffic_light_green.png";
      break;
    }
    case 3:
    {
      image_file_path = assets_folder + "/images/traffic_light_all_gray.png";
      break;
    }
    default:
      break;
  }

  // This right now goes to the assets/map/traffic_light_green.png, and it should be elsewhere, but is fine for now
  cv::Mat behavior_image = cv::imread( image_file_path );

  if( behavior_image.empty() )
  {
    return {};
  }

  return std::optional<cv::Mat> { behavior_image };
}
 
std::optional<cv::Mat> fetch_vehicle_state_image(const std::string &asset_storage_path, const float &distance_to_nearest_participant)
{
  std::string image_file_path = "";

  if ( distance_to_nearest_participant > 20.0 )
    image_file_path = asset_storage_path+ "/images/autonomous_car_on_road.png";

  if ( distance_to_nearest_participant < 20.0 )
    image_file_path = asset_storage_path + "/images/autonomous_car_on_road_other_vehicle_seen.png";

  if ( distance_to_nearest_participant < 10.0 )
    image_file_path = asset_storage_path + "/images/autonomous_car_on_road_other_vehicle_middle.png";

  if ( distance_to_nearest_participant < 5.0 )
    image_file_path = asset_storage_path + "/images/autonomous_car_on_road_other_vehicle_close.png";

  cv::Mat behavior_image = cv::imread( image_file_path );

  if( behavior_image.empty() )
  {
    return {};
  }

  return std::optional<cv::Mat> { behavior_image };
}

sensor_msgs::msg::Image image_to_msg(const cv::Mat& image)
{
  std_msgs::msg::Header header;  // optional, set timestamp/frame_id if needed
  // header.stamp = now();
  header.frame_id = "camera";
  sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

  return *msg;

}

} // namespace behavior
} // namespace visualizer
} // namespace adore
