#ifndef SHOOT_CLIENT_H
#define SHOOT_CLIENT_H

#include "roborts_msgs/FricWhl.h"
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/armor_detecte_result.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/RobotHeat.h"


void FriWhl_ctrl(bool s);

void Shoot_ctrl(int shoot_mode, int shoot_num);




#endif