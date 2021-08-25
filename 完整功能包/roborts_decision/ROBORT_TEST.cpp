#include <ros/ros.h>
#include <time.h>
#include "executor/chassis_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/F1_behavior.h"
#include "example_behavior/F2_behavior.h"
#include "example_behavior/F3_behavior.h"
#include "example_behavior/F4_behavior.h"
#include "example_behavior/F5_behavior.h"
#include "example_behavior/F6_behavior.h"
#include "example_behavior/swing_behavior.h"
#include "example_behavior/shoot_behavior.h"
#include "example_behavior/wait_behavior.h"
#include "example_behavior/ready_behavior.h"
#include "example_behavior/observe_behavior.h"

#include "std_msgs/Bool.h"

int shoot_flag = 0, count = 0, which_buff = 0, getbuff_act = 0, which_buff_flag = 0;
bool Detection_flag = false;


void Command();
char command = '0';

int main(int argc, char **argv) {

  ros::init(argc, argv, "Roborts_Test");

  std::string full_path = ros::package::getPath("roborts_decision") + "/config/red_wing.prototxt";
  
  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);

  roborts_decision::BackBootAreaBehavior  back_boot_area_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::EscapeBehavior       escape_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::PatrolBehavior       patrol_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::WaitBehavior       wait_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ShootBehavior       shoot_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::F1Behavior       f1_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::F2Behavior       f2_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::F3Behavior       f3_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::F4Behavior       f4_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::F5Behavior       f5_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::F6Behavior       f6_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SwingBehavior       swing_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ReadyBehavior       ready_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ObserveBehavior       observe_behavior(chassis_executor, blackboard, full_path);


  auto command_thread= std::thread(Command);
  ros::Rate rate(10);
  while(ros::ok()){
    ros::spinOnce();
    switch (command) {
      //back to boot area
      case '1':
        back_boot_area_behavior.Run();
        break;
        //patrol
      case '2':
        patrol_behavior.Run();
        break;
        //chase.
      case '3':
        swing_behavior.Run();
        break;
        //search
      case '4':
        wait_behavior.Run();
        break;
        //escape.
      case '5':
        shoot_behavior.Run();
        break;
        //goal.
       case '6':
         observe_behavior.Run();
         break;     
     case 'q':
        f1_behavior.Run();
        break;       
      case 'w':
        f2_behavior.Run();
        break;
      case 'e':
        f3_behavior.Run();
        break;
      case 'r':
        f4_behavior.Run();
        break;            
      case 't':
        f5_behavior.Run();
        break; 
      case 'y':
        f6_behavior.Run();
        break;            
      case 27:
        if (command_thread.joinable()){
          command_thread.join();
        }
        return 0;
      default:
        break;
    }
    rate.sleep();
  }
  return 0;
}

void Command() {

  while (command != 27) {
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "*********************************please send a command********************************" << std::endl;
    std::cout << "1: back boot area behavior" << std::endl
              << "2: patrol behavior" << std::endl
              << "3: swing_behavior" << std::endl
              << "4: wait_behavior behavior" << std::endl
              << "5: shoot_behavior behavior" << std::endl
               << "6: observe_behavior" << std::endl
              << "q: f1_behavior" << std::endl
              << "w: f2_behavior" << std::endl
              << "e: f3_behavior" << std::endl
              << "r: f4_behavior" << std::endl
              << "t: f5_behavior" << std::endl
              << "y: f6_behavior" << std::endl
              << "esc: exit program" << std::endl;
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5'&& command != '6'
    && command != '7'&& command != '8'&& command != '9'&& command != '-' && command != 27) {
      std::cout << "please input again!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

  }
}



