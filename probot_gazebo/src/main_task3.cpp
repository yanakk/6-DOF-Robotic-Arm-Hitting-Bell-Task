/*
#include  <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include"../inc/cxz_robot_arm.h"
double new_joint_angle[6];
void cxz::SpeedCurve(const double t,double (&expected_speed)[6]);

const double pi=3.1415926;
void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  new_joint_angle[0]=msg->position[0];
  new_joint_angle[1]=msg->position[1];
  new_joint_angle[2]=msg->position[2];
  new_joint_angle[3]=msg->position[3];
  new_joint_angle[4]=msg->position[4];
  new_joint_angle[5]=msg->position[5];
}

int main(int argc,char *argv[])
{
// 初始化节点
    ros::init(argc, argv, "vel_control");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    ROS_INFO_STREAM("start");
    ros::Publisher speed_pub=node_handle.advertise<std_msgs::Float32MultiArray>("speed_chatter",1000);
    std_msgs::Float32MultiArray init_vel;
    init_vel.data.push_back(0);
    init_vel.data.push_back(0);
    init_vel.data.push_back(0);
    init_vel.data.push_back(0);
    init_vel.data.push_back(0);
    init_vel.data.push_back(0);
    sleep(1);
//
    double t,target_speed[6],joint_speed[6];
    cxz::RobotArm robotarm;
// 设置接收频率为50HZ
    ros::Subscriber sub = node_handle.subscribe("/probot_anno/joint_states", 100, jointstatesCallback);
    ros::Rate loop_rate(50);
    ros::Time begin = ros::Time::now();    
    while (ros::ok()  && ((ros::Time::now()-begin).toSec()<12.1))
    {
 // 接收关节角位移
        ros::spinOnce();
        t=(ros::Time::now()-begin).toSec();
// 获取当前时刻笛卡尔空间的速度期望
        cxz::SpeedCurve(t,target_speed);
// 反馈更新程序中的机械臂状态
        robotarm.UpdateAngle(new_joint_angle);
// 求解当前关节角速度
        robotarm.SpeedControl(target_speed,joint_speed);
// 发布速度信息
init_vel.data.at(0)=joint_speed[0]*30*180/pi;
init_vel.data.at(1)=joint_speed[1]*205*180/(3*pi);
init_vel.data.at(2)=joint_speed[2]*50*180/pi;
init_vel.data.at(3)=joint_speed[3]*125*180/(2*pi);
init_vel.data.at(4)=joint_speed[4]*125*180/(2*pi);
init_vel.data.at(5)=joint_speed[5]*200*180/(9*pi);
        speed_pub.publish(init_vel);
        loop_rate.sleep();
    }

    return 0;
}
*/