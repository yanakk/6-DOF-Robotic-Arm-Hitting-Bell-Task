/*
#include  <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <math.h>
#include"../inc/cxz_robot_arm.h"
const double pi=3.1415926;
void cxz::SpeedCurve(const double t,double (&expected_speed)[6]);
void cxz::TrackSolution(const std::vector<double> & point_value,std::vector<double *> & solution);
double cxz::ReadVelocityFromSolution(const std::vector<double *> & solution,const double & t);
double cxz::ReadValueFromSolution(const std::vector<double *> & solution,const double & t);
void cxz::CubicSpline(const std::vector<double> & point_value,std::vector<double *> & solution);
double end_angle[6];
void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  end_angle[0]=msg->position[0];
  end_angle[1]=msg->position[1];
  end_angle[2]=msg->position[2];
  end_angle[3]=msg->position[3];
  end_angle[4]=msg->position[4];
  end_angle[5]=msg->position[5];
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
//  定义变量
    double target_pose[6],target_angle[6];
    double target_joint_angle[6][4];
    std::vector<double> point_value;
    std::vector<double *> solution[6];
    double target_pose_array[4][6]={{0.2289,0,0.454,1.57,0,0},
                               {0.3,0.25,0.322,1.57,-1.57,0},
                               {0.3,0.1,0.172,1.57,-1.57,0},
                               {0.3,-0.1,0.122,1.57,-1.57,0}};
    cxz::RobotArm robotarm;
// 解逆运动学
    for(size_t i=0;i<4;i++){
        for(size_t j=0;j<6;j++)
            target_pose[j]=target_pose_array[i][j];
        robotarm.SolveTheTargetPoint(target_pose,target_angle);
        for(size_t j=0;j<6;j++)
            target_joint_angle[j][i]=target_angle[j];        
    }
// 轨迹规划
    for(size_t i=0;i<6;i++){
        point_value.clear();
        for(size_t j=0;j<4;j++)
            point_value.push_back(target_joint_angle[i][j]);
        cxz::TrackSolution(point_value,solution[i]);
    }
    double temp,largest;
    for(size_t i=0;i<3;i++){
        largest=solution[0][i][4];
        for(size_t j=0;j<6;j++){
            temp=solution[j][i][4];
            if(largest<temp)
                largest=temp;
        }
        for(size_t j=0;j<6;j++){
            solution[j][i][4]=largest;
        }
    }
    for(size_t i=0;i<6;i++){
        point_value.clear();
        for(size_t j=0;j<4;j++)
            point_value.push_back(target_joint_angle[i][j]);
        cxz::CubicSpline(point_value,solution[i]);
    }

// 发布角速度控制
    ros::Subscriber sub = node_handle.subscribe("/probot_anno/joint_states", 100, jointstatesCallback);
    double t=0;
    ros::Rate loop_rate(50);
    ros::Time begin = ros::Time::now();    
    while (ros::ok())
    {
        t=(ros::Time::now()-begin).toSec();
// 更新发布的速度
init_vel.data.at(0)=cxz::ReadVelocityFromSolution(solution[0],t)*30*180/pi;
init_vel.data.at(1)=cxz::ReadVelocityFromSolution(solution[1],t)*205*180/(3*pi);
init_vel.data.at(2)=cxz::ReadVelocityFromSolution(solution[2],t)*50*180/pi;
init_vel.data.at(3)=cxz::ReadVelocityFromSolution(solution[3],t)*125*180/(2*pi);
init_vel.data.at(4)=cxz::ReadVelocityFromSolution(solution[4],t)*125*180/(2*pi);
init_vel.data.at(5)=cxz::ReadVelocityFromSolution(solution[5],t)*200*180/(9*pi);
        speed_pub.publish(init_vel);
// 判断是否到达终点
        if((init_vel.data.at(0) || init_vel.data.at(1) || init_vel.data.at(2) || init_vel.data.at(3) || init_vel.data.at(4) || init_vel.data.at(5))==0 && t>10)
            break;
        loop_rate.sleep();
    }
 
// 打印终点的位姿
    ros::spinOnce();
    robotarm.UpdateAngle(end_angle);
    std::cout<<" duration: "<<t<<std::endl;
    robotarm.PrintTailPositionAndPosture();   
    return 0;
}
*/