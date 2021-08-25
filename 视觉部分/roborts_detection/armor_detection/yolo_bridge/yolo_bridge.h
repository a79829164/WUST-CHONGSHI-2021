/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_DETECTION_ARMOR_DETECTION_YOLO_BRIDGE_H
#define ROBORTS_DETECTION_ARMOR_DETECTION_YOLO_BRIDGE_H

//system include
#include <vector>
#include <list>

#include <opencv2/opencv.hpp>

#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"

#include "cv_toolbox.h"

#include "../armor_detection_base.h"

#include "yolo_bridge.h"

#include "roborts_msgs/armor_detecte_result.h"

#include <mutex>

namespace roborts_detection {

using roborts_common::ErrorCode;
using roborts_common::ErrorInfo;


/**
 * @brief This class achieved functions that can help to detect armors of RoboMaster vehicle.
 */
class YoloBridge : public ArmorDetectionBase {
 public:
  YoloBridge(std::shared_ptr<CVToolbox> cv_toolbox);
  /**
   * @brief Loading parameters from .prototxt file.
   */
  void LoadParam() override;
  /**
   * @brief The entrance function of armor detection.
   * @param translation Translation information of the armor relative to the camera.
   * @param rotation Rotation information of the armor relative to the camera.
   */
  ErrorInfo DetectArmor(bool &detected, cv::Point3f &target_3d) override;

  void SetThreadState(bool thread_state) override;
  /**
   * @brief Destructor
   */
  ~YoloBridge() final;
private:
  ErrorInfo error_info_;

  uint8_t enemy_color_;

  bool thread_running_;
  //ros
  ros::NodeHandle nh;

  //result sub
  ros::Subscriber armor_sub;

  //result temp
  bool detected_result;
  cv::Point3f target_3d_result;
  std::mutex detected_result_mux;

  void armorDetectedResultCallback(const roborts_msgs::armor_detecte_result::ConstPtr& msg);
};

roborts_common::REGISTER_ALGORITHM(ArmorDetectionBase, "yolo_bridge", YoloBridge, std::shared_ptr<CVToolbox>);

} //namespace roborts_detection

#endif // ROBORTS_DETECTION_ARMOR_DETECTION_YOLO_BRIDGE_H
