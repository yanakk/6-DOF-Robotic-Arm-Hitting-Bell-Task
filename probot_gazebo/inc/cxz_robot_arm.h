#ifndef  _ROBOT_ARM_PARAMETER_
#define _ROBOT_ARM_PARAMETER_

#include <vector>
#define PI 3.1415926535
#define ZERO 0.00001
namespace cxz
{
    struct Position{
        double coord[3];
        double posture_maxtrix[3][3];
    };
    
    struct Posture{
        double posture_maxtrix[3][3];
        double posture_angle[3];
    };

    class RobotArm 
    {
        size_t joint_number;
        std::vector<double> joint_angle;
        std::vector<Position> joint_position;   
        Posture tail_posture;
        std::vector<double *>  mdh_parameter;
        std::vector<double *>  sdh_parameter; 
    public:
        RobotArm();
        int UpdateAngle(const double (&new_joint_angle)[6]);
        int SolveTheTargetPoint(const double (&target_pose)[6],double (&target_angle)[6]);
        int SpeedControl(const double (&target_speed)[6],double (&joint_speed)[6]);
        void PrintTailPositionAndPosture(void);
        void PrintJointAngle(void);
    private:
        void UpdateMDH(void);
        void TailPostureMatrixToAngle(void);
        void ComputePositionAndPosture(void);
    }; 

    void ComputePostureAngleByMatrixXYZ(const double (&posture_matrix)[3][3],double  (&posture_angle)[3]);
    void ComputePostureAngleByMatrixZYZ(const double (&posture_matrix)[3][3],double  (&posture_angle)[3]);
    void ComputePostureMatrixByAngle(double (&posture_matrix)[3][3],const double  (&posture_angle)[3]);
    void ComputeQuaternion(const double (&posture_matrix)[3][3],double  (&quaternion_angle)[4]);
    void ComputeInverseKinematics(const double(&target_pose)[6], double(&angle_solve)[2][6]);
    int ChoseSolution(const double(&now_joint_angle)[6],const double(&angle_solve)[2][6], std::vector<double *> &angle_solution);
    void SpeedCurve(const double t,double(&expected_speed)[6]);
    void TrackSolution(const std::vector<double> & point_value,std::vector<double *> & solution);
    void TrackSolution(const std::vector<double> & point_value,std::vector<double *> & solution,size_t index);
    void TrackInitialTime(const std::vector<double> & point_value,std::vector<double *> & solution);
    void TrackInitialTime(const std::vector<double> & point_value,std::vector<double *> & solution,size_t index);
    void CubicSpline(const std::vector<double> & point_value,std::vector<double *> & solution);
    int IsTimeTooShort(const double * s);
    int IsTimeTooShort(const double * s, size_t index);
    int IsTimeTooLong(const double * s);
    int IsTimeTooLong(const double * s,size_t index);
    int PositiveAdjustTime(std::vector<double *> & solution);
    int PositiveAdjustTime(std::vector<double *> & solution,size_t index);
    int NegativeAdjustTime(std::vector<double *> & solution);
    int NegativeAdjustTime(std::vector<double *> & solution,size_t index);
    double ReadValueFromSolution(const std::vector<double *> & solution,const double & t);
    double ReadVelocityFromSolution(const std::vector<double *> & solution,const double & t);
    double ReadAcceleratedVelocityFromSolution(const std::vector<double *> & solution,const double & t);
    size_t MyPathGenerate(const double (& fixed_position)[3],std::vector<double *> & path_array);
    size_t RingPath(std::vector<double *> & path_array);
}
#endif
