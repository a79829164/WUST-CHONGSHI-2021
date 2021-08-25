#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################
	
# 该例程将发布turtle1/cmd_vel话题，消息类型geometry_msgs::Twist

import rospy
from geometry_msgs.msg import Twist
import time
from roborts_msgs.msg import GameZoneArray
from roborts_msgs.msg import GameStatus
from std_msgs.msg import Int16MultiArray
# ROS节点初始化
rospy.init_node('base_move', anonymous=True)
# 速度指令发布者
velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


# 位置标记，是返回起始点的依据
master_pose = 0
# master_pose =4 表示第二次buff更新后由于前方有车，呆在家不动
# master_psoe = 3 表示2区域有车，在B2点的基础上转向45度
# master_pose = 2 表示6区域有车，在B2点的基础上转向90度
# master_pose =1 表示1区域有车，我方车转移到H点（B点正后方）


# 从机对应的哨岗话题
def car_info_subscription():
    index = True
    m = 0
    list = [0, 0, 0]
    while(index):
        try:
            carinfo_msg_1 = rospy.wait_for_message("/car_position_info2",Int16MultiArray,timeout=3)
            rospy.loginfo("Subcribe car Info(bird2):  color:%d  x:%d  y:%d",carinfo_msg_1.data[0], carinfo_msg_1.data[1], carinfo_msg_1.data[2])
            return carinfo_msg_1.data
        except:
            rospy.loginfo("no info!")
            index=False
            return list

def get_enemy_coordinate():
    enemy_coordinate = car_info_subscription()
    list = [0, 0, 0]
    item = True
    while(item):
        if(enemy_coordinate[0]==1):
            item = False
            return enemy_coordinate
        else:
            return list

def game_status():
    game_status_msg = rospy.wait_for_message("/game_status",GameStatus,timeout=None)
    rospy.loginfo("game status :%d",game_status_msg.game_status)   
    return game_status_msg

def turn_around():
    for i in range(2):
        route(0,0,1.57,3)
        route(0,0,-1.57,3)


def get_game_zone_status():
    gamezone_msg =rospy. wait_for_message("/game_zone_array_status", GameZoneArray, timeout=None)
    rospy.loginfo("received the game zone position:%s",gamezone_msg.zone)
    return gamezone_msg

               
def first_behavior():
    gamezone = get_game_zone_status()
    F1 = gamezone.zone[0]
    F2 = gamezone.zone[1]
    F3 = gamezone.zone[2]
    # 初始运动行为先不考虑哨兵信息，默认最终位置为B2,正面向敌方
    if( F3==1 or F3==4):
        # A->B->B2
        route(1.4,0,0,6)    
        route(-0.8,0,0,3) 
        route(0,-0.5,0,3) 
    else:
        # A->B2
        route(1.4,0,0,6)
        route(0,-0.5,0,3) 

def second_behavior():
    gamezone_1 = get_game_zone_status()
    F1_ = gamezone_1.zone[0]
    F2_ = gamezone_1.zone[1]
    F3_ = gamezone_1.zone[2]
    enemy_coordinate = get_enemy_coordinate()
    if(enemy_coordinate[2]<-100 and enemy_coordinate[1]<-70.5):
        # 如果前方区域有敌方车辆，呆着不动
        route(0,0,0,10)
        master_pose=4
    elif(F3_==3 or F3_==4):
        # A->B->B2
        route(1.72,0,0,5)    
        route(-1,0,0,3) 
        route(0,-0.5,0,3) 
    else:
         # A->B2
        route(1.72,0,0,4)
        route(0,-0.5,0,3)




def route(linear_x,linear_y,ang,index):
    m = 0
    vel_msg = Twist()
    for i in range(index):
        if i == index-1:
            vel_msg.linear.x=0
            vel_msg.linear.y=0
            vel_msg.angular.z=0
        else:
            vel_msg.linear.x = linear_x
            vel_msg.linear.y = linear_y
            vel_msg.angular.z = ang
            m+=1

        time.sleep(0.5)
        velocity_pub.publish(vel_msg)
        rospy.loginfo("Publsh  velocity command[X:%0.2f m/s,Y:%0.2f m/s,angular:%0.2f rad/s],times=%f",vel_msg.linear.x,vel_msg.linear.y,vel_msg.angular.z,m)
def main():
    global master_pose
    template_1 = True
    template_2 = True
    template_3 = True
    while (template_1):
        gamestatus = game_status()
        if (gamestatus.game_status == 4):
            rospy.loginfo("game begin!")
            # 执行first_behavior行为
            first_behavior()
            # 获取哨岗信息，判断从车对应的敌方主车坐标
            enemy_position = get_enemy_coordinate()
            #  如果敌方主车在１号区域，则运动到B2旁边的点一对一
            # 但是要考虑敌方车辆是否会过段时间才过来，待定
            # 并且还要自身坐标的接收
            if ((enemy_position[1] < -130.5) and (enemy_position[2] < 202)):
                time.sleep(2)
                route(0, 0.5, 0, 3)
                # 设置最终标志位为G
                master_pose = 1
            # 如果2有车
            elif (enemy_position[1] > -130.5 and (enemy_position[2] > 0 and enemy_position[2] < 202)):
                route(0, 0, -0.785, 3)
                master_pose = 2
            elif (enemy_position[1] > -130.5 and (enemy_position[2] < 0 and enemy_position[2] > -202)):
                route(0, 0, -1.57, 3)
                master_pose = 3
            else:
                turn_around()

            # 执行完后重新获取比赛状态,判断最新的比赛状态是否在预估范围内，一般不会有变化
            while (template_2):
                # 不断获取比赛状态
                gamestatus_2 = game_status()
                if (gamestatus_2.remaining_time < 124):
                    rospy.loginfo("back home!")
                    # 检测最终位置, 执行回家行为,考虑后方是否有敌方车辆
                    enemy_position_1 = get_enemy_coordinate()
                    if ((master_pose == 1) and (enemy_position_1[1] < -70.5 and enemy_position_1[2] < -100)):
                        # 先转圈圈
                        route(0, 0, -3.14 / 3, 5)
                        # 后方有车就不用回去了
                        template_1 = False
                    elif (master_pose == 1):
                        # back home
                        route(-1.02, 0, 0, 6)
                    # 同样考虑后面是否有车
                    elif ((master_pose == 2) and (enemy_position_1[1] < -70.5 and enemy_position_1[2] < -100)):
                        # 有车转圈圈
                        route(0, 0, 1.57, 3)
                    elif (master_pose == 2):
                        # 没车回家
                        route(0, 0, 0.785, 3)
                        route(-1.02, 0, 0, 6)
                    elif ((master_pose == 3) and (enemy_position_1[1] < -70.5 and enemy_position_1[2] < -100)):
                        # 有车转圈圈
                        route(0, 0, 1.57, 3)
                    elif (master_pose == 3):
                        # 没车回家
                        route(0, 0, 1.57, 3)
                        route(0, 0.6, 0, 3)
                        route(-1.02, 0, 0, 6)
                    else:
                        route(0, 0.6, 0, 3)
                        route(-1.02, 0, 0, 6)
                    template_2 = False
                else:
                    rospy.loginfo("turn around!")
                    turn_around()

            # 执行second_behavior行为
            second_behavior()
            enemy_position_2 = get_enemy_coordinate()
            if ((enemy_position_2[1] < -130.5) and (enemy_position_2[2] < 202)):
                time.sleep(2)
                route(0, 0.5, 0, 3)
                # 设置最终标志位为G
                master_pose = 1
                # 如果2有车
            elif (enemy_position_2[1] > -130.5 and (enemy_position_2[2] > 0 and enemy_position_2[2] < 202)):
                route(0, 0, -0.785, 3)
                master_pose = 2
            elif (enemy_position_2[1] > -130.5 and (enemy_position_2[2] < 0 and enemy_position_2[2] > -202)):
                route(0, 0, -1.57, 3)
                master_pose = 3
            else:
                turn_around()
                # 第三次获取比赛状态
            while (template_3):
                # 不断获取比赛状态
                gamestatus_4 = game_status()
                if (gamestatus_4.remaining_time < 64):
                    rospy.loginfo("back home again!")
                    enemy_position_3 = get_enemy_coordinate()
                    if ((master_pose == 1) and (enemy_position_3[1] < -70.5 and enemy_position_3[2] < -100)):
                        # 先转圈圈
                        route(0, 0, -3.14 / 3, 5)
                        # 后方有车就不用回去了
                        template_1 = False
                    elif (master_pose == 1):
                        # back home
                        route(-1.02, 0, 0, 6)
                    # 同样考虑后面是否有车
                    elif ((master_pose == 2) and (enemy_position_3[1] < -70.5 and enemy_position_3[2] < -100)):
                        # 有车转圈圈
                        route(0, 0, 1.57, 3)
                    elif (master_pose == 2):
                        # 没车回家
                        route(0, 0, 0.785, 3)
                        route(-1.02, 0, 0, 6)
                    elif ((master_pose == 3) and (enemy_position_3[1] < -70.5 and enemy_position_3[2] < -100)):
                        # 有车转圈圈
                        route(0, 0, 1.57, 3)
                    elif (master_pose == 3):
                        # 没车回家
                        route(0, 0, 1.57, 3)
                        route(0, 0.6, 0, 3)
                        route(-1.02, 0, 0, 6)
                    else:
                        route(0, 0.6, 0, 3)
                        route(-1.02, 0, 0, 6)
                    template_3 = False
                else:
                    rospy.loginfo("turn around again!")
                    turn_around()
                    # 执行first_behavior行为
            second_behavior()
            turn_around()
            template_1 = False
        else:
            rospy.loginfo("waiting ......")


if __name__ == '__main__':
    main()
