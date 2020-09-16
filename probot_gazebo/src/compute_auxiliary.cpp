#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <iostream>
#include "../inc/cxz_robot_arm.h"

void cxz::ComputePostureAngleByMatrixXYZ(const double (&posture_matrix)[3][3],double  (&posture_angle)[3])
{
     double x,y,z;
    y=atan2(-posture_matrix[2][0],pow(posture_matrix[0][0]*posture_matrix[0][0]+posture_matrix[1][0]*posture_matrix[1][0],0.5));
    z=atan2(posture_matrix[1][0]/cos(y),posture_matrix[0][0]/cos(y));
    x=atan2(posture_matrix[2][1]/cos(y),posture_matrix[2][2]/cos(y));

    posture_angle[0]=x;   
    posture_angle[1]=y;
    posture_angle[2]=z;
}

void cxz::ComputePostureAngleByMatrixZYZ(const double (&posture_matrix)[3][3],double  (&posture_angle)[3])
{
	double z1,y,z2;
	y = atan2(pow(posture_matrix[2][0]* posture_matrix[2][0]+ posture_matrix[2][1] * posture_matrix[2][1],0.5), posture_matrix[2][2]);
	z1 = atan2(posture_matrix[1][2]/sin(y), posture_matrix[0][2]/sin(y));
	z2 = atan2(posture_matrix[2][1]/sin(y),-posture_matrix[2][0]/sin(y));

	posture_angle[0] = z1;
	posture_angle[1] = y;
	posture_angle[2] = z2;
}

void cxz::ComputePostureMatrixByAngle(double (&posture_matrix)[3][3],const double  (&posture_angle)[3])
{
        using namespace Eigen;
        Matrix3f X,Y,Z,R;
        X<< 1,0,0,
                 0,cos(posture_angle[0]),-sin(posture_angle[0]),
                 0,sin(posture_angle[0]),cos(posture_angle[0]);
        Y<< cos(posture_angle[1]),0,sin(posture_angle[1]),
                 0,1,0,
                -sin(posture_angle[1]),0,cos(posture_angle[1]);
        Z<< cos(posture_angle[2]),-sin(posture_angle[2]),0,
                 sin(posture_angle[2]),cos(posture_angle[2]),0,
                 0,0,1;
        R=Z*Y*X;

        for(size_t i=0;i<3;i++){
            for(size_t j=0;j<3;j++){
                posture_matrix[i][j]=R(i,j);
            }
        }
}

void cxz::ComputeQuaternion(const double (&posture_matrix)[3][3],double  (&quaternion_angle)[4])
{
    quaternion_angle[3]=0.5*pow(1+posture_matrix[0][0]+posture_matrix[1][1]+posture_matrix[2][2],0.5);
    quaternion_angle[0]=(posture_matrix[2][1]-posture_matrix[1][2])/(4*quaternion_angle[3]);
    quaternion_angle[1]=(posture_matrix[0][2]-posture_matrix[2][0])/(4*quaternion_angle[3]);
    quaternion_angle[2]=(posture_matrix[1][0]-posture_matrix[0][1])/(4*quaternion_angle[3]);
}

void cxz::SpeedCurve(const double t,double(&expected_speed)[6])
{
    expected_speed[1]=expected_speed[3]=expected_speed[4]=expected_speed[5]=0;
    if(t>0 && t<=4){
        expected_speed[0]=0.005*t*1.414214/2;
        expected_speed[2]=-expected_speed[0];
    }
    else if(t>4 && t<=7){
        expected_speed[0]=0.01414214;
        expected_speed[2]=-expected_speed[0];
    }
    else if(t>7 && t<=12){
        expected_speed[0]=0.01414214-0.004*(t-7)*1.414214/2;
        expected_speed[2]=-expected_speed[0];
    }
    else
        expected_speed[0]=expected_speed[2]=0;
}

size_t cxz::MyPathGenerate(const double (& fixed_position)[3],std::vector<double *> & path_array)
{
    cxz::RobotArm robotarm;
    double pose[6],angle[6];
    double *p=NULL;
    for(size_t i=0;i<100;i++){
// 按规律生成轨迹点
        pose[0]=fixed_position[0];
        pose[1]=fixed_position[1];
        pose[2]=fixed_position[2];
        pose[3]=2*i*PI/100+PI/2;
        pose[4]=0;
        pose[5]=0;
 // 判断机器人是否 能达到这个姿态，如果可以就保留轨迹点，如果不行轨迹生成结束
        if(robotarm.SolveTheTargetPoint(pose,angle)==0){
            p=new double[6];
            p[0]=pose[0];
            p[1]=pose[1];
            p[2]=pose[2];
            p[3]=pose[3];
            p[4]=pose[4];
            p[5]=pose[5];
            path_array.push_back(p);
        }
        else
            break;
    }
// 返回轨迹点的个数
    return path_array.size();
}