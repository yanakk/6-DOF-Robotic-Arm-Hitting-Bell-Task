#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <vector>
#include "../inc/cxz_robot_arm.h"

double RANGEOFMOVEMENT[6][2] = { { -3.14,3.14 },{ -2.01,2.01 },{ -0.69,3.83 },{ -3.14,3.14 },{ -0.78,3.92 },{ -3.14,3.14 } };
void cxz::ComputeInverseKinematics(const double(&target_pose)[6], double(&angle_solution_temp)[2][6])
{
	using namespace Eigen;
	double posture_angle[3];
	for (int i = 0; i<3; i++)
		posture_angle[i] = target_pose[i + 3];
	posture_angle[0] = posture_angle[0] + PI / 2;

	double posture_matrix[3][3];
	ComputePostureMatrixByAngle(posture_matrix, posture_angle);

	double x = target_pose[0] - posture_matrix[0][2] * 0.055;
	double y = target_pose[1] - posture_matrix[1][2] * 0.055;
	double z = target_pose[2] - posture_matrix[2][2] * 0.055;

	double angle1 = atan2(y, x);
	double a = pow(x*x + y*y, 0.5);
	double b = z - 0.284;
	double distance = pow(a*a + b*b, 0.5);
	double angle3 = acos((0.225*0.225 + 0.2289*0.2289 - distance*distance) / (2 * 0.225*0.2289)) - PI / 2;
	double temp = atan2(b, a);
	double angle2 = acos((0.225*0.225 - 0.2289*0.2289 + distance*distance) / (2 * 0.225*distance));
	angle2 = angle2 + temp - PI / 2;

	double parameter[4][4] = { { 0, 0, 0.284, angle1 },{ PI / 2, 0.0, 0.0, (PI / 2 + angle2) },{ 0,0.225,0,angle3 },{ PI / 2,0,0.2289,0 } };
	Matrix3f M = MatrixXf::Identity(3, 3);
	Matrix3f T, R_06, R_46;
	for (size_t i = 0; i<4; i++) {
		T << cos(parameter[i][3]), -sin(parameter[i][3]), 0,
			sin(parameter[i][3])*cos(parameter[i][0]), cos(parameter[i][3])*cos(parameter[i][0]), -sin(parameter[i][0]),
			sin(parameter[i][3])*sin(parameter[i][0]), cos(parameter[i][3])*sin(parameter[i][0]), cos(parameter[i][0]);
		M = M*T;
	}
	R_06 << posture_matrix[0][0], posture_matrix[0][1], posture_matrix[0][2],
		posture_matrix[1][0], posture_matrix[1][1], posture_matrix[1][2],
		posture_matrix[2][0], posture_matrix[2][1], posture_matrix[2][2];
	R_46 = (M.inverse())*R_06;
	for (size_t i = 0; i<3; i++) {
		for (size_t j = 0; j<3; j++)
			posture_matrix[i][j] = R_46(i, j);
	}
	ComputePostureAngleByMatrixZYZ(posture_matrix, posture_angle);

	angle_solution_temp[0][0] = angle1;
	angle_solution_temp[0][1] = angle2;
	angle_solution_temp[0][2] = angle3;
	angle_solution_temp[0][3] = posture_angle[0];
	angle_solution_temp[0][4] = posture_angle[1] + PI / 2;
	angle_solution_temp[0][5] = posture_angle[2];

	angle_solution_temp[1][0] = angle1;
	angle_solution_temp[1][1] = angle2;
	angle_solution_temp[1][2] = angle3;
	angle_solution_temp[1][3] = posture_angle[0] +PI;
	angle_solution_temp[1][4] = -posture_angle[1] + PI / 2;
	angle_solution_temp[1][5] = posture_angle[2] +PI;
}

int cxz::ChoseSolution(const double(&now_joint_angle)[6],const double(&angle_solution_temp)[2][6], std::vector<double *> &angle_solution)
{
	double temp_angle[2][6];
	double *p=NULL;
	int flag = 0;
	for (size_t i = 0; i < 2; i++) {
		flag = 0;
		for (size_t j = 0; j < 6; j++) {
			temp_angle[i][j] = angle_solution_temp[i][j];
			if (temp_angle[i][j] < RANGEOFMOVEMENT[j][0])
				temp_angle[i][j] = temp_angle[i][j] + 2 * PI;
			else if(temp_angle[i][j] > RANGEOFMOVEMENT[j][1])
				temp_angle[i][j] = temp_angle[i][j] - 2 * PI;
			if (temp_angle[i][j] >= RANGEOFMOVEMENT[j][0] && temp_angle[i][j] <= RANGEOFMOVEMENT[j][1])
				;
			else {
				flag = 1;
				break;
			}
		}
		if (flag == 0) {
			p = new double[6];
			for (size_t j = 0; j < 6; j++)
				p[j] = temp_angle[i][j];
			angle_solution.push_back(p);
		}
	}


	if (angle_solution.size() == 0)
		return 1;
	else if (angle_solution.size() == 1)
		return 0;
	else {
		double value[2];
		for (size_t i = 0; i < 2; i++) {
			value[i] = 0;
			for (size_t j = 0; j < 6; j++)
				value[i] = value[i] + fabs(angle_solution[i][j] - now_joint_angle[j]);
		}
		if (value[1] > value[0])
			return 0;
		else {
			p = angle_solution[0];
			angle_solution[0] = angle_solution[1];
			angle_solution[1] = p;
			return 0;
		}
	}

}
