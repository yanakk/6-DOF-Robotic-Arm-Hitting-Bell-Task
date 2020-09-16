/*
#include  <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include <iostream>
#include"../inc/cxz_robot_arm.h"

const double pi=3.1415926;
int main(int argc,char *argv[])
{
// 初始化节点
    ros::init(argc, argv, "pos_control");
    ros::AsyncSpinner spinner(1);
    ROS_INFO_STREAM("start");
	ros::NodeHandle node_handle;  
	//创建名为position_pub 的位置publisher  
    ros::Publisher position_pub = node_handle.advertise<std_msgs::Float32MultiArray>("position_chatter", 1000);  

std_msgs::Float32MultiArray postion_msg;  
postion_msg.data.push_back(0.0);  
postion_msg.data.push_back(0.0);  
postion_msg.data.push_back(0.0);  
postion_msg.data.push_back(0.0);  
postion_msg.data.push_back(0.0);  
postion_msg.data.push_back(0.0);  
postion_msg.data.push_back(0.0);  
      sleep(3);

//正运动学代码
 // 定义目标 关节角
double new_angle[6]={0.505893,-1.3877,0.213872,0.173959,0.990619,0.351396}; 
//    double new_angle[6]={0.927,-0.687,-0.396,0,1.083,0.927};
//   double new_angle[6]={0.322,-0.855,-0.021,0,0.877,0.322};
//   double new_angle[6]={-0.322,0.636,-0.011,0,0.647,-0.322};
// 新建对象
    cxz::RobotArm robotarm;
 // 更新关节角
    robotarm.UpdateAngle(new_angle);
// 打印末端位置
    robotarm.PrintTailPositionAndPosture();
postion_msg.data.at(0) = float(new_angle[0] * 30 * 180 / pi);  
postion_msg.data.at(1) = float(new_angle[1] * 205 * 180 / (3 * pi));  
postion_msg.data.at(2) = float(new_angle[2] * 50 * 180 / pi);  
postion_msg.data.at(3) = float(new_angle[3] * 125 * 180 / (2 * pi));  
postion_msg.data.at(4) = float(new_angle[4] * 125 * 180 / (2 * pi));  
postion_msg.data.at(5) = float(new_angle[5] * 200 * 180 / (9 * pi));  
postion_msg.data.at(6) = 1200.0;//设定速度  
position_pub.publish(postion_msg);  
sleep(10);
                    
new_angle[0]=-0.689591;
new_angle[1]= -1.56673;
new_angle[2]= 0.722835;
new_angle[3]= -0.165291;
new_angle[4]= 0.683038;
new_angle[5]= -0.574958;

postion_msg.data.at(0) = float(new_angle[0] * 30 * 180 / pi);  
postion_msg.data.at(1) = float(new_angle[1] * 205 * 180 / (3 * pi));  
postion_msg.data.at(2) = float(new_angle[2] * 50 * 180 / pi);  
postion_msg.data.at(3) = float(new_angle[3] * 125 * 180 / (2 * pi));  
postion_msg.data.at(4) = float(new_angle[4] * 125 * 180 / (2 * pi));  
postion_msg.data.at(5) = float(new_angle[5] * 200 * 180 / (9 * pi));  
postion_msg.data.at(6) = 1200.0;//设定速度  
position_pub.publish(postion_msg);  
sleep(10);


// 逆运动学代码
 //目标点位姿
//   double target_pose[6]={0.2,0.2,0.2007,1.57,-1.57,0}; 
//   double target_pose[6]={0.15,0.2,0.2007,0,0,0};
    double target_pose[6]={0.3,0,0.122,1.57,0,0};
 //存放关节角
    double new_angle[6];
 //新建对象 
    cxz::RobotArm robotarm;
 //求解逆运动学，结果存放到new_angle
    robotarm.SolveTheTargetPoint(target_pose,new_angle);
// 更新关节角
    robotarm.UpdateAngle(new_angle);
// 打印关节角
    robotarm.PrintJointAngle();
//打印程序仿真器计算出的末端位置
    robotarm.PrintTailPositionAndPosture();
 // 发送信息给机械臂
postion_msg.data.at(0) = float(new_angle[0] * 30 * 180 / pi);  
postion_msg.data.at(1) = float(new_angle[1] * 205 * 180 / (3 * pi));  
postion_msg.data.at(2) = float(new_angle[2] * 50 * 180 / pi);  
postion_msg.data.at(3) = float(new_angle[3] * 125 * 180 / (2 * pi));  
postion_msg.data.at(4) = float(new_angle[4] * 125 * 180 / (2 * pi));  
postion_msg.data.at(5) = float(new_angle[5] * 200 * 180 / (9 * pi));  
postion_msg.data.at(6) = 1200.0;//设定速度  
position_pub.publish(postion_msg);  
sleep(10);

     for(size_t i=0;i<6;i++)
            init_pos.data.at(i) = new_angle[i];      
    pos_pub.publish(init_pos);
    ROS_INFO_STREAM("published");

    return 0;
}
*/