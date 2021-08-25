#include <ros/ros.h>
#include <iostream>
#include "shoot_client.h"

ros::ServiceClient shoot_client;
ros::ServiceClient FricWheel_client;
bool detection_or_not;

unsigned int cont = 0;

int stop_shoot = 0;
int single_shoot = 1;
int continue_shoot = 2;

void game_callback(const roborts_msgs::GameStatus::ConstPtr &game_msg)
{
    if(game_msg -> game_status == 4)
    {
        FriWhl_ctrl(true);
    }
    else
    {
        FriWhl_ctrl(false); 
    }

}

/**************************************************************************
void shoot_heat_callback(const roborts_msgs::RobotHeat::ConstPtr &heat_msg)
{
    if(heat_msg -> shooter_heat == 0)
    {
        Shoot_ctrl(stop_shoot,5);
    }
    if(heat_msg -> shooter_heat < 240)
    {
        Shoot_ctrl(single_shoot,5);
    }
    if(heat_msg -> shooter_heat > 240)
    {
        Shoot_ctrl(continue_shoot,1);
    }
}
**************************************************************************/

void shoot_callback(const roborts_msgs::armor_detecte_result::ConstPtr &detecte_msg)
{

  ROS_INFO("subcribe the detection result:%d",detecte_msg->is_detected);
  if(detecte_msg->is_detected)
  {
      ROS_INFO("shoot!");
      Shoot_ctrl(2,3);
      Shoot_ctrl(stop_shoot,0);
  }
}

void FriWhl_ctrl(bool s)
{
  roborts_msgs::FricWhl fric_srv;

  fric_srv.request.open = s;
  FricWheel_client.call(fric_srv);
/*
  if (FricWheel_client.call(fric_srv) && fric_srv.response.received == 1 && s == true)
    {
      ROS_INFO("Open the fri-wheel!");
    }
  else if (s == false)
    {
      ROS_INFO("Close the fri-wheel!");
    }
  else if(fric_srv.response.received == 0)
    {
      ROS_ERROR("Failed to call fri-wheel service!");
    }
*/
}

void Shoot_ctrl(int shoot_mode, int shoot_num)
{
  roborts_msgs::ShootCmd shoot_srv;

  shoot_srv.request.mode = shoot_mode;
  shoot_srv.request.number = shoot_num;
  shoot_client.call(shoot_srv);
/*
  if (shoot_client.call(shoot_srv))
    {
      ROS_INFO("Shoot now!");
    }
  else{
      ROS_ERROR("Failed to call shoot client");
    }
*/
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "shoot_client");


    ros::NodeHandle nh;

    ros::service::waitForService("cmd_fric_wheel");
    FricWheel_client = nh.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
    ros::service::waitForService("cmd_shoot");
    shoot_client = nh.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");

    ros::Subscriber game_info = nh.subscribe<roborts_msgs::GameStatus>("game_status",10,game_callback);
    ros::Subscriber detection_info = nh.subscribe<roborts_msgs::armor_detecte_result>("armor_position_info",10,shoot_callback);
    //ros::Subscriber heat_info = nh.subscribe<roborts_msgs::RobotHeat>("robot_heat",10,shoot_heat_callback);

    ros::spin();
    return 0;
}
