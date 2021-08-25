#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include "yolo_bridge.h"

#include "timer/timer.h"
#include "io/io.h"

#include <fstream>

namespace roborts_detection
{

  YoloBridge::YoloBridge(std::shared_ptr<CVToolbox> cv_toolbox) : ArmorDetectionBase(cv_toolbox)
  {
    thread_running_ = false;
    target_3d_result.x = 0;
    target_3d_result.y = 0;
    target_3d_result.z = 0;
    detected_result = false;
    finished = false;
    LoadParam();
    error_info_ = ErrorInfo(roborts_common::OK);
    armor_sub = nh.subscribe("/armor_position_info", 10, &YoloBridge::armorDetectedResultCallback, this);
  }

  void YoloBridge::LoadParam()
  {
    std::string file_name = ros::package::getPath("roborts_detection") + \
      "/armor_detection/yolo_bridge/config/yolo_bridge.prototxt";
    std::ifstream file(file_name);

    char line[100];
    file.getline(line, 100);
    file.close();


    if (std::strcmp(line, "enemy_color: BLUE") == 0)
    {
      enemy_color_ = 1;
    }
    else
    {
      enemy_color_ = 0;
    }

    ROS_INFO("load enemy name: %d", enemy_color_);
  }

  ErrorInfo YoloBridge::DetectArmor(bool &detected, cv::Point3f &target_3d)
  {
    ROS_INFO("DetectArmor");
    while (true) {
        finished_mux.lock();
        if (finished) {
            finished = false;
            break;
        }
        finished_mux.unlock();
    }
    finished_mux.unlock();
    
    detected_result_mux.lock();
    detected = detected_result;
    target_3d.x = target_3d_result.x;
    target_3d.y = target_3d_result.y;
    target_3d.z = target_3d_result.z;
    detected_result_mux.unlock();
  }

  void YoloBridge::SetThreadState(bool thread_state)
  {
    thread_running_ = thread_state;
  }

  YoloBridge::~YoloBridge()
  {
  }

  void YoloBridge::armorDetectedResultCallback(const roborts_msgs::armor_detecte_result::ConstPtr& msg)
  {
    if (msg->color == enemy_color_)
    {
      detected_result_mux.lock();
      detected_result = msg->is_detected;
      target_3d_result.x = msg->armor_x;
      target_3d_result.y = msg->armor_y;
      target_3d_result.z = msg->armor_z;
      detected_result_mux.unlock();
    }
    finished_mux.lock();
    finished = true;
    ROS_INFO("armorDetectedResultCallback");
    finished_mux.unlock();
  }

} //namespace roborts_detection
