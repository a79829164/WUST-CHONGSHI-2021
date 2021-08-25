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
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
//#include <std_msgs/Int16MultiArray>

#include "roborts_msgs/ArmorDetectionAction.h"
#include "roborts_msgs/armor_detecte_result.h"

#include "roborts_msgs/ArmorPos.h"
#include "roborts_msgs/ArmorsPos.h"
#include "roborts_msgs/ShooterCmd.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/RobotHeat.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/RobotShoot.h"
#include "roborts_msgs/ProjectileSupply.h"
#include "roborts_msgs/SupplierStatus.h"
#include "roborts_msgs/BonusStatus.h"
#include "roborts_msgs/GameZoneArray.h"
#include "roborts_msgs/GameRobotBullet.h"
#include "roborts_msgs/GameRobotHP.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"

#include "communication.h"


namespace roborts_decision{

struct Threshold{
    float near_dist;
    float near_angle;
    float heat_upper_bound;
    float detect_dist;

};

struct DecisionInfoPool{
  //self
  int buff_point;
  int red_hp_recovery;
  int red_bullet_supply, blue_hp_recovery, blue_bullet_supply, disable_shooting, disable_movement;
  int zone_array[6], zone_array_active[6];
  bool activate;
  int R_HP_point , R_Bullet_point , B_HP_point  , B_Bullet_point , DisShooting_point , DisMove_point;
  int remain_hp_b1, remain_hp_b2, remain_hp_r1, remain_hp_r2;
  int remain_bullet_b1, remain_bullet_b2, remain_bullet_r1, remain_bullet_r2;
  int deep_camera_red, deep_camera_blue, deep_camera_color;
  float deep_camera_armor_x, deep_camera_armor_y, deep_camera_armor_z;
  bool deep_camera_is_detected;

  int remaining_time;
  int times_to_supply;
  int times_to_buff;
  int game_status;
  int shoot_hz;
  bool is_begin;
  bool is_master;
  bool team_blue;
  bool has_buff;
  bool has_ally;
  bool has_my_enemy;
  bool has_ally_enemy;
  bool has_first_enemy;
  bool has_second_enemy;
  bool can_shoot;
  bool can_dodge;
  bool is_supplying;
  bool is_shielding;
  bool got_last_enemy;
  bool use_refree;

  // ally enemy part
  bool has_ally_first_enemy;
  bool has_ally_second_enemy;

  bool is_hitted;
  bool is_chase;
  bool valid_camera_armor;
  bool valid_front_camera_armor;
  bool valid_back_camera_armor;
  int remain_hp;
  int frequency;
  float speed;

};

class Blackboard {
 public:
 int shoot_flag = 0, count = 0;

bool Detection_flag = false;

  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  explicit Blackboard(const std::string &proto_file_path):
      enemy_detected_(false),
      armor_detection_actionlib_client_("armor_detection_node_action", true){

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    // Enemy fake pose
    ros::NodeHandle rviz_nh("/move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);

    ros::NodeHandle nh;
             
    //robot_status_sub_ = nh.subscribe<roborts_msgs::RobotStatus>("robot_status", 10, &Blackboard::RobotStatusCallback, this);
    //robot_shoot_sub_ = nh.subscribe<roborts_msgs::RobotShoot>("robot_shoot", 10, &Blackboard::RobotShootCallback, this);
    game_status_sub_ = nh.subscribe<roborts_msgs::GameStatus>("game_status", 10, &Blackboard::GameStatusCallback, this);
    game_zone_sub_ = nh.subscribe<roborts_msgs::GameZoneArray>("game_zone_array_status", 10, &Blackboard::GameZoneArrayCallback, this);
    game_robort_hp_sub_ = nh.subscribe<roborts_msgs::GameRobotHP>("game_robot_hp", 10, &Blackboard::GameRobotHPCallback, this);
    game_robort_bullet_sub_ = nh.subscribe<roborts_msgs::GameRobotBullet>("game_robot_bullet", 10, &Blackboard::GameRobotBulletCallback, this);
    game_robort_deep_camera_sub_ = nh.subscribe<roborts_msgs::armor_detecte_result>("armor_position_info",10,&Blackboard::armor_detecte_result_callback, this);

    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

    if (!decision_config.simulate()){

      armor_detection_actionlib_client_.waitForServer();

      ROS_INFO("Armor detection module has been connected!");

      armor_detection_goal_.command = 1;
      armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                                 boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));
    }


  }

  ~Blackboard() = default;


  // Enemy
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->detected){
      enemy_detected_ = true;
      ROS_INFO("Find Enemy!");

      tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
      camera_pose_msg = feedback->enemy_pos;

      double distance = std::sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x +
          camera_pose_msg.pose.position.y * camera_pose_msg.pose.position.y);
      double yaw = atan(camera_pose_msg.pose.position.y / camera_pose_msg.pose.position.x);

      //camera_pose_msg.pose.position.z=camera_pose_msg.pose.position.z;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,
                                                              0,
                                                              yaw);
      camera_pose_msg.pose.orientation.w = quaternion.w();
      camera_pose_msg.pose.orientation.x = quaternion.x();
      camera_pose_msg.pose.orientation.y = quaternion.y();
      camera_pose_msg.pose.orientation.z = quaternion.z();
      poseStampedMsgToTF(camera_pose_msg, tf_pose);

      tf_pose.stamp_ = ros::Time(0);
      try
      {
        tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
        tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);

        if(GetDistance(global_pose_msg, enemy_pose_)>0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2){
          enemy_pose_ = global_pose_msg;

        }
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("tf error when transform enemy pose from camera to map");
      }
    } else{
      enemy_detected_ = false;
      ROS_INFO("Don't Find Enemy!");
    }

  }

  geometry_msgs::PoseStamped GetEnemy() const {
    return enemy_pose_;
  }

  bool IsEnemyDetected() const{
    ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    return enemy_detected_;
  }

  // Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
    new_goal_ = true;
    goal_ = *goal;
  }

  geometry_msgs::PoseStamped GetGoal() const {
    return goal_;
  }

  bool IsNewGoal(){
    if(new_goal_){
      new_goal_ =  false;
      return true;
    } else{
      return false;
    }
  }

  bool IsNewEnemy(){
    bool goal=IsEnemyDetected();
    if(goal){
      goal =  false;
      return true;
    } else{
      return false;
    }
  }

  void GameStatusCallback(const roborts_msgs::GameStatus::ConstPtr &msg){
      info.remaining_time = msg->remaining_time;
      info.is_begin = bool(msg->game_status == 4);
      info.game_status = msg->game_status;
  }

  void GameRobotBulletCallback(const roborts_msgs::GameRobotBullet::ConstPtr &bu){
      info.remain_bullet_b1 = bu->blue1;
      info.remain_bullet_b2 = bu->blue2;
      info.remain_bullet_r1 = bu->red1;
      info.remain_bullet_r2 = bu->red2;
  }

    void GameRobotHPCallback(const roborts_msgs::GameRobotHP::ConstPtr &hp){
      info.remain_hp_b1 = hp->blue1;
      info.remain_hp_b2 = hp->blue2;
      info.remain_hp_r1 = hp->red1;
      info.remain_hp_r2 = hp->red2;
  }

void armor_detecte_result_callback(const roborts_msgs::armor_detecte_result::ConstPtr &deep_camera){
      info.deep_camera_red = deep_camera->RED;
      info.deep_camera_blue = deep_camera->BLUE;
      info.deep_camera_color = deep_camera->color;
      info.deep_camera_is_detected = deep_camera->is_detected;
      info.deep_camera_armor_x = deep_camera->armor_x;
      info.deep_camera_armor_y = deep_camera->armor_y;
      info.deep_camera_armor_z = deep_camera->armor_z;  
          ROS_INFO("is_detected:%d, x:%d, y:%d, z:%d\n", info.deep_camera_is_detected, info.deep_camera_armor_x, info.deep_camera_armor_y, info.deep_camera_armor_z);   
}


  void GameZoneArrayCallback(const roborts_msgs::GameZoneArray::ConstPtr &zo){
      for (int i=0; i<6; i++) info.zone_array[i] = zo->zone[i].type ;

      for (int m=0; m<6; m++) info.zone_array_active[m] = zo->zone[m].active;
        
  }













  bool IsInStuckArea(){
      geometry_msgs::PoseStamped curPose = GetRobotMapPose();
      double yaw = tf::getYaw(curPose.pose.orientation);
      float cur_x = curPose.pose.position.x, cur_y = curPose.pose.position.y;
      float x[4], y[4];
      float deg_30 = 30 / 180 * 3.1415926;
      float d = 0.3;
      x[0] = std::max(0.0, std::min(7.99, cur_x + d * std::cos(yaw + deg_30)));
      y[0] = std::max(0.0, std::min(4.99, cur_y + d * std::sin(yaw + deg_30)));
      x[1] = std::max(0.0, std::min(7.99, cur_x + d * std::cos(yaw - deg_30)));
      y[1] = std::max(0.0, std::min(4.99, cur_y + d * std::sin(yaw - deg_30)));
      x[2] = std::max(0.0, std::min(7.99, cur_x - d * std::cos(yaw + deg_30)));
      y[2] = std::max(0.0, std::min(4.99, cur_y - d * std::sin(yaw + deg_30)));
      x[3] = std::max(0.0, std::min(7.99, cur_x - d * std::cos(yaw - deg_30)));
      y[3] = std::max(0.0, std::min(4.99, cur_y - d * std::sin(yaw - deg_30)));
      unsigned int mx, my;
      for (int i=0; i<4; i++){
          GetCostMap2D()->World2Map(x[i], y[i], mx, my);
          if (GetCostMap2D()->GetCost(mx, my) <253)
            return false;
      }

      // In stuck area
      
      const double sqrt2 = 1.414;
      double cost;
      double c_x, c_y;
      double int_x[] = {-d, -d/sqrt2, 0, d/sqrt2, d, d/sqrt2, 0, -d/sqrt2};
      double int_y[] = {0, -d/sqrt2, -d, d/sqrt2, 0, d/sqrt2, d, -d/sqrt2};
      int u_x, u_y;
      double acc_angle = 0.0;
      double cur_angle = 0.0;
      double count = 0;
      for (int i=0; i<=7; i++){
            c_x = cur_x + int_x[i];
            c_y = cur_y + int_y[i];
            GetCostMap2D()->World2MapWithBoundary(c_x, c_y, u_x, u_y);
            cost = GetCostMap2D()->GetCost(u_x, u_y);
            if (cost >= 253){
                cur_angle = std::atan2(int_y[i], int_x[i] + 0.00001);
                cur_angle = cur_angle >0? cur_angle : cur_angle + 2 * 3.1415926;
                acc_angle += cur_angle;
                count++;
            }
                      
          
      }
      acc_angle = acc_angle / count;
      acc_angle = acc_angle - 3.1415926;
      acc_angle = acc_angle < -3.1415926 ? acc_angle + 2 * 3.1415926 : acc_angle;

      // in case of rotate in a roll.
      c_x = cur_x + d * std::cos(acc_angle);
      c_y = cur_y + d * std::sin(acc_angle);
      GetCostMap2D()->World2MapWithBoundary(c_x, c_y, u_x, u_y);
      cost = GetCostMap2D()->GetCost(u_x, u_y);
      if (cost >=253){
          acc_angle = acc_angle >0 ? acc_angle - 3.1415926 : acc_angle + 3.1415926;
      }
    //   printf("acc_angle: %f\n", acc_angle * 180 / 3.14);

      // get yaw
      acc_angle = acc_angle - yaw;
      double vx, vy;
      vx = cos(acc_angle);
      vy = sin(acc_angle);
      geometry_msgs::Twist tw;
      tw.linear.x = vx;
      tw.linear.y = vy;
      tw.linear.z = 0;
      // has sent cmd_vel
      cmd_vel_pub_.publish(tw);
      std::cout<<"WAIT" << std::endl;

      return true;
  }






  /*---------------------------------- Tools ------------------------------------------*/

  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

  const geometry_msgs::PoseStamped GetRobotMapPose() {
    UpdateRobotPose();
    return robot_map_pose_;
  }

  const std::shared_ptr<CostMap> GetCostMap(){
    return costmap_ptr_;
  }

  const CostMap2D* GetCostMap2D() {
    return costmap_2d_;
  }

  const unsigned char* GetCharMap() {
    return charmap_;
  }

  DecisionInfoPool info;
  Threshold threshold;

 private:
  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();

    robot_tf_pose.frame_id_ = "base_link";
    robot_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }
  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! Enenmy detection
  ros::Subscriber enemy_sub_;

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;

  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  geometry_msgs::PoseStamped enemy_pose_;
  bool enemy_detected_;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

ros::ServiceClient shoot_client;
ros::ServiceClient FricWheel_client;

  //! robot map pose
  geometry_msgs::PoseStamped robot_map_pose_;
  ros::Subscriber enemy_pose_sub;
  ros::Subscriber  game_robort_hp_sub_,  game_robort_bullet_sub_, game_robort_deep_camera_sub_;
  ros::Subscriber robot_status_sub_, robot_shoot_sub_, robot_heat_sub_, game_status_sub_, supply_sub_, buff_sub_, game_zone_sub_;
  ros::Publisher shoot_pub_, ally_pub_, fusion_pub_, dodge_pub_, supply_pub_, cmd_vel_pub_;
  roborts_msgs::GameZone Buff_Status[6];


  int remain_hp;

};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H