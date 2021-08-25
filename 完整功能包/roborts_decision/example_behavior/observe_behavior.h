#ifndef OBSERVE_BEHAVIOR_H
#define OBSERVE_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class ObserveBehavior {
 public:
  ObserveBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard) {


    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {

    auto executor_state = Update();

    std::cout << "state: " << (int)(executor_state) << std::endl;

    if (executor_state != BehaviorState::RUNNING) {

      if (observe_goals_.empty()) {
        ROS_ERROR("patrol goal is empty");
        return;
      }

      std::cout << "send goal" << std::endl;
      chassis_executor_->Execute(observe_goals_[0]);

    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    observe_goals_.resize(1);
            if (decision_config.isblue()){
      observe_goals_[0].header.frame_id = "map";
      observe_goals_[0].pose.position.x = decision_config.blue().observe_point(0).x();
      observe_goals_[0].pose.position.y = decision_config.blue().observe_point(0).y();
      observe_goals_[0].pose.position.z = decision_config.blue().observe_point(0).z();

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.blue().observe_point(0).roll(),
                                                              decision_config.blue().observe_point(0).pitch(),
                                                              decision_config.blue().observe_point(0).yaw());
      observe_goals_[0].pose.orientation.x = quaternion.x();
      observe_goals_[0].pose.orientation.y = quaternion.y();
      observe_goals_[0].pose.orientation.z = quaternion.z();
      observe_goals_[0].pose.orientation.w = quaternion.w();
            }
            else
            {
                    observe_goals_[0].header.frame_id = "map";
      observe_goals_[0].pose.position.x = decision_config.red().observe_point(0).x();
      observe_goals_[0].pose.position.y = decision_config.red().observe_point(0).y();
      observe_goals_[0].pose.position.z = decision_config.red().observe_point(0).z();

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.red().observe_point(0).roll(),
                                                              decision_config.red().observe_point(0).pitch(),
                                                              decision_config.red().observe_point(0).yaw());
      observe_goals_[0].pose.orientation.x = quaternion.x();
      observe_goals_[0].pose.orientation.y = quaternion.y();
      observe_goals_[0].pose.orientation.z = quaternion.z();
      observe_goals_[0].pose.orientation.w = quaternion.w();
            }

    return true;
  }

  ~ObserveBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! patrol buffer
  std::vector<geometry_msgs::PoseStamped> observe_goals_;


};
}

#endif