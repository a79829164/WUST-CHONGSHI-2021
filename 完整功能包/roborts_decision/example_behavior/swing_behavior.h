#ifndef ROBORTS_SWING_BEHAVIOR_H
#define ROBORTS_SWING_BEHAVIOR_H

#include <ros/ros.h>
#include <time.h>
#include <unistd.h>
#include <geometry_msgs/Twist.h>
#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"
int swing_run=1;

namespace roborts_decision {
class SwingBehavior {
 public:
    SwingBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard) {


    // if (!LoadParam(proto_file_path)) {
    //   ROS_ERROR("%s can't open file", __FUNCTION__);
    // }

  }

  void Run() {

    auto executor_state = Update();

    std::cout << "state: " << (int)(executor_state) << std::endl;

    if (executor_state != BehaviorState::RUNNING) {

	ros::NodeHandle nh;
    ros::Publisher turtle_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate loop_rate(10);
	    // 初始化geometry_msgs::Twist类型的消息
		geometry_msgs::Twist vel_msg;


        sleep(1);
		vel_msg.angular.z = 0.5;
    		ROS_INFO("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", 
				vel_msg.linear.x, vel_msg.angular.z);
  	turtle_vel_pub.publish(vel_msg);

for (int i = 0; i < 3; i++)
  {
        sleep(1);
		vel_msg.angular.z = -1;
    		ROS_INFO("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", 
				vel_msg.linear.x, vel_msg.angular.z);
  	turtle_vel_pub.publish(vel_msg);
        sleep(1);
		vel_msg.angular.z = 1;
	    // 发布消息
		turtle_vel_pub.publish(vel_msg);
		ROS_INFO("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", 
				vel_msg.linear.x, vel_msg.angular.z);
  }
        sleep(1);
		vel_msg.angular.z = -0.5;
    		ROS_INFO("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", 
				vel_msg.linear.x, vel_msg.angular.z);
  	turtle_vel_pub.publish(vel_msg);
	    // 按照循环频率延时
	    loop_rate.sleep();
    }
  }

  void Cancel() {
      swing_run = 0;
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }


  ~SwingBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

};
}

#endif 
