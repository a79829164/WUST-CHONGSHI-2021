#ifndef ROBORTS_F5_GETBUFF_BEHAVIOR_H
#define ROBORTS_F5_GETBUFF_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class F5Behavior {
 public:
    F5Behavior(ChassisExecutor* &chassis_executor,
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

      if (BuffPoint_.empty()) {
        ROS_ERROR("buff goal is empty");
        return;
      }

    //chassis_executor_->Execute(BuffPoint_);
      chassis_executor_->Execute(BuffPoint_[0]);
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

        BuffPoint_.resize(1);
    //action
               if (decision_config.isblue()){
                  BuffPoint_[0] .header.frame_id = "map";
                  BuffPoint_[0].pose.position.x = decision_config.blue().buffpoint_f5(0).x();
                  BuffPoint_[0].pose.position.y = decision_config.blue().buffpoint_f5(0).y();
                  BuffPoint_[0].pose.position.z = decision_config.blue().buffpoint_f5(0).z();

                  tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.blue().buffpoint_f5(0).roll(),
                  decision_config.blue(). buffpoint_f5(0).pitch(),
                  decision_config.blue(). buffpoint_f5(0).yaw());
                  BuffPoint_[0].pose.orientation.x = quaternion.x();
                  BuffPoint_[0].pose.orientation.y = quaternion.y();
                  BuffPoint_[0].pose.orientation.z = quaternion.z();
                  BuffPoint_[0].pose.orientation.w = quaternion.w();
            }
            else
            {
                  BuffPoint_[0] .header.frame_id = "map";
                  BuffPoint_[0].pose.position.x = decision_config.red().buffpoint_f5(0).x();
                  BuffPoint_[0].pose.position.y = decision_config.red().buffpoint_f5(0).y();
                  BuffPoint_[0].pose.position.z = decision_config.red().buffpoint_f5(0).z();

                  tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.red().buffpoint_f5(0).roll(),
                  decision_config.red(). buffpoint_f5(0).pitch(),
                  decision_config.red(). buffpoint_f5(0).yaw());
                  BuffPoint_[0].pose.orientation.x = quaternion.x();
                  BuffPoint_[0].pose.orientation.y = quaternion.y();
                  BuffPoint_[0].pose.orientation.z = quaternion.z();
                  BuffPoint_[0].pose.orientation.w = quaternion.w();
            }
    return true;
  }

  ~F5Behavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  std::vector<geometry_msgs::PoseStamped> BuffPoint_;
};
}

#endif 
