#include <vector>
#include <math.h>
#include"../inc/cxz_robot_arm.h"
double MAX_V_Ring[6]={0.9,0.52,0.8,0.8,0.95,2}; //速度上限
double MAX_V_LIMIT_Ring[6]={0.9,0.52,0.8,0.8,0.95,2}; //超过这个值，则认为已经达到了较大速度
double MAX_A_Ring[6]={5.5,3,1.5,0.8,1.2,10}; //加速度上限
double MAX_A_LIMIT_Ring[6]={5.5,3,1.5,0.8,1.2,10}; //超过这个值，则认为已经达到了较大加速度
#define STEP_RING 0.001         //时间优化时每步长
#define ADJUST_NUMBER_RING 10         //正反时间调整次数

void cxz::TrackSolution(const std::vector<double> & point_value,std::vector<double *> & solution,size_t index)
 {
// 初始化时间
    TrackInitialTime(point_value,solution,index);
// 求解三次样条
    CubicSpline(point_value,solution);
// 时间优化
    for(size_t i=0;i<ADJUST_NUMBER_RING;i++){
// 反向调整时间
        while(NegativeAdjustTime(solution,index)){
            CubicSpline(point_value,solution);
        }
// 正向调整时间
        while(PositiveAdjustTime(solution,index)){
            CubicSpline(point_value,solution);
        }
    }
    CubicSpline(point_value,solution);
 }

size_t cxz::RingPath(std::vector<double *> & path_array)
{
    double *p[7];
// p0
    p[0]=new double[6];
    p[0][0]=0.2289;
    p[0][1]=0;
    p[0][2]=0.454;
    p[0][3]=1.57;
    p[0][4]=0;
    p[0][5]=0;
//p1
    p[1]=new double[6];
    p[1][0]=0.26;
    p[1][1]=0.15;
    p[1][2]=0.06;
    p[1][3]=1.57;
    p[1][4]=0.2;
    p[1][5]=0;
//p2
    p[2]=new double[6];
    p[2][0]=0.40;
    p[2][1]=0.02;
    p[2][2]=0.16;
    p[2][3]=1.57;
    p[2][4]=-0.3;
    p[2][5]=0;
//p3
    p[3]=new double[6];
    p[3][0]=0.28;
    p[3][1]=-0.24;
    p[3][2]=0.06;
    p[3][3]=1.57;
    p[3][4]=0.2;
    p[3][5]=0;
//
    path_array.push_back(p[0]);  
    path_array.push_back(p[1]);  
    path_array.push_back(p[2]);
    path_array.push_back(p[3]);
     for(size_t i=0;i<21;i++) {
         path_array.push_back(p[2]);
         path_array.push_back(p[1]); 
         path_array.push_back(p[2]);
         path_array.push_back(p[3]);
     }
    return (path_array.size());
}

int cxz::IsTimeTooShort(const double * s, size_t index)
{
    double temp;
    temp=-s[2]/(3*s[3]);
    if(temp>=0 && temp<=s[4]){
        if(fabs(3*s[3]*pow(temp,2)+2*s[2]*temp+s[1])<=MAX_V_Ring[index]
           && fabs(3*s[3]*pow(0,2)+2*s[2]*0+s[1])<=MAX_V_Ring[index] 
           && fabs(3*s[3]*pow(s[4],2)+2*s[2]*s[4]+s[1])<=MAX_V_Ring[index]
           && fabs(2*s[2])<=MAX_A_Ring[index] && fabs(2*s[2]+6*s[3]*s[4])<=MAX_A_Ring[index])
           return 0;
        else
           return 1;
    }
    else{
        if(fabs(3*s[3]*pow(0,2)+2*s[2]*0+s[1])<=MAX_V_Ring[index] 
           && fabs(3*s[3]*pow(s[4],2)+2*s[2]*s[4]+s[1])<=MAX_V_Ring[index]
           && fabs(2*s[2])<=MAX_A_Ring[index] && fabs(2*s[2]+6*s[3]*s[4])<=MAX_A_Ring[index])
           return 0;
        else
           return 1;
    }
}
int cxz::IsTimeTooLong(const double * s,size_t index)
{
    double temp;
    temp=-s[2]/(3*s[3]);
    if(temp>=0 && temp<=s[4]){
        if(fabs(3*s[3]*pow(temp,2)+2*s[2]*temp+s[1])<=MAX_V_LIMIT_Ring[index] 
           && fabs(3*s[3]*pow(0,2)+2*s[2]*0+s[1])<=MAX_V_LIMIT_Ring[index] 
           && fabs(3*s[3]*pow(s[4],2)+2*s[2]*s[4]+s[1])<=MAX_V_LIMIT_Ring[index]
           && fabs(2*s[2])<=MAX_A_LIMIT_Ring[index]  && fabs(2*s[2]+6*s[3]*s[4])<=MAX_A_LIMIT_Ring[index] )
           return 1;
        else
           return 0;
    }
    else{
        if(fabs(3*s[3]*pow(0,2)+2*s[2]*0+s[1])<=MAX_V_LIMIT_Ring[index]  
           && fabs(3*s[3]*pow(s[4],2)+2*s[2]*s[4]+s[1])<=MAX_V_LIMIT_Ring[index] 
           && fabs(2*s[2])<=MAX_A_LIMIT_Ring[index]  && fabs(2*s[2]+6*s[3]*s[4])<=MAX_A_LIMIT_Ring[index] )
           return 1;
        else
           return 0;
    }
}
int cxz::PositiveAdjustTime(std::vector<double *> & solution,size_t index)
{
   size_t solution_size=solution.size();
    int flag=0;
    for(size_t i=0;i<solution_size;i++){
        if(IsTimeTooShort(solution[i],index)){
            solution[i][4]=solution[i][4]+STEP_RING;
            flag=1;
        }
    }
    if(flag==0)
       return 0;
    else
       return 1;
}
int cxz::NegativeAdjustTime(std::vector<double *> & solution,size_t index)
{
    size_t solution_size=solution.size();
    int flag=0;
    for(size_t i=0;i<solution_size;i++){
        if(IsTimeTooLong(solution[i],index)){
            solution[i][4]=solution[i][4]-STEP_RING;
            flag=1;
        }
    }
    if(flag==0)
       return 0;
    else
       return 1;
}
void cxz::TrackInitialTime(const std::vector<double> & point_value,std::vector<double *> & solution,size_t index)
{
    double *p=NULL;
    size_t m_size=point_value.size()-1;
    for(size_t i=0;i<m_size;i++){
        p=new double[5];
        solution.push_back(p);
    }
    for(size_t i=0;i<m_size;i++){
        solution[i][4]=fabs((point_value[i+1]-point_value[i])/(MAX_V_Ring[index] ));
    }
}
/*
// p4
    p[4]=new double[6];
    p[4][0]=0.312021;
    p[4][1]=0.0840781;
    p[4][2]=0.287662;
    p[4][3]=1.96458;
    p[4][4]=0.0436399;
    p[4][5]=1.166;
// p5    
    p[5]=new double[6];
    p[5][0]=0.285725;
    p[5][1]=0.0385064;
    p[5][2]=0.380408;
    p[5][3]=1.74071;
    p[5][4]=0.107272;
    p[5][5]=0.605911;
// p6
    p[6]=new double[6];
    p[6][0]=0.302586;
    p[6][1]=0.125316;
    p[6][2]=0.184197;
    p[6][3]=2.12996;
    p[6][4]=-0.167972;
    p[6][5]=1.6341;
*/