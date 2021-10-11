#include <vector>
#include <algorithm>
#include <cmath>
#include <iomanip>

#ifndef __TRANSFORMATION__
#define __TRANSFORMATION__

#define JOINT 6
#define SIZE 4
#define PI 3.141592
#define COS(x) cos(x*PI/180)
#define SIN(x) sin(x*PI/180)
#define ATAN2(y,x) (atan2(y,x)*180/PI)
#define NSIN(x) (sin(x*PI/180)) != 0 ? (-sin(x*PI/180)):0

void Homogeneous_forward_transform(std::vector<std::vector<double>> before_matrix, std::vector<std::vector<double>> trans_matrix, std::vector<std::vector<double>>& after_matrix);
void Homogeneous_inverse_matrix(std::vector<std::vector<double>> before_matrix, std::vector<std::vector<double>>& inverse_matrix);
void Homogeneous_rpy_matrix(double* position, double* rpy_angle, std::vector<std::vector<double>>& after_matrix);
std::vector<double> Homogeneous_inverse_kinematic(std::vector<std::vector<double>> robot_base_hand_matrix,double* normal,int num_of_joint);

#endif __TRANSFORMATION__
