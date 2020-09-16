#include  <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include"../inc/cxz_robot_arm.h"
using namespace cxz;
double end_angle[6];
const double pi=3.1415265;
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
// 节点初始化
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
// 变量定义
    double target_pose[6],target_angle[6];
    double * target_joint_angle[6];
    std::vector<double> point_value;
    std::vector<double *> solution[6];
    cxz::RobotArm robotarm;
// 输入运动的点轨迹
    std::vector<double *>  path_array;
    size_t size_path=cxz::RingPath(path_array);
// 求解逆运动学并优化得到速度规划
    for(size_t i=0;i<6;i++)
        target_joint_angle[i]=new double[size_path];
    for(size_t i=0;i<size_path;i++){
        for(size_t j=0;j<6;j++)
            target_pose[j]=path_array[i][j];
        robotarm.SolveTheTargetPoint(target_pose,target_angle);
        for(size_t j=0;j<6;j++)
            target_joint_angle[j][i]=target_angle[j];        
    }
    for(size_t i=0;i<5;i++){
        point_value.clear();
        for(size_t j=0;j<size_path;j++)
            point_value.push_back(target_joint_angle[i][j]);
        cxz::TrackSolution(point_value,solution[i],i);
    }
// 使每个关节在两点间上拥有相同的速度    
    double temp,largest,sum=0;
    for(size_t i=0;i<(size_path-1);i++){
        largest=solution[0][i][4];
        for(size_t j=0;j<5;j++){
            temp=solution[j][i][4];
            if(largest<temp)
                largest=temp;
        }
        for(size_t j=0;j<5;j++){
            solution[j][i][4]=largest;
        }
        sum=sum+largest;
    }
        std::cout<<"预计总时长："<<sum<<std::endl;
// 重解三样条
    for(size_t i=0;i<5;i++){
        point_value.clear();
        for(size_t j=0;j<(size_path);j++)
            point_value.push_back(target_joint_angle[i][j]);
        cxz::CubicSpline(point_value,solution[i]);
    }
// 发布信息
    ros::Subscriber sub = node_handle.subscribe("/probot_anno/joint_states", 100, jointstatesCallback);
    double t=0;
    ros::Rate loop_rate(20);
    ros::Time begin = ros::Time::now();    
    while (ros::ok())
    {
        t=(ros::Time::now()-begin).toSec();
init_vel.data.at(0)=cxz::ReadVelocityFromSolution(solution[0],t)*30*180/pi;
init_vel.data.at(1)=cxz::ReadVelocityFromSolution(solution[1],t)*205*180/(3*pi);
init_vel.data.at(2)=cxz::ReadVelocityFromSolution(solution[2],t)*50*180/pi;
init_vel.data.at(3)=cxz::ReadVelocityFromSolution(solution[3],t)*125*180/(2*pi);
init_vel.data.at(4)=cxz::ReadVelocityFromSolution(solution[4],t)*125*180/(2*pi);
speed_pub.publish(init_vel);
        if((init_vel.data.at(0) || init_vel.data.at(1) || init_vel.data.at(2) || init_vel.data.at(3) || init_vel.data.at(4) || init_vel.data.at(5))==0 && t>10)
            break;
        loop_rate.sleep();
    }
    return 0;
}
