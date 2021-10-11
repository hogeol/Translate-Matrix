#include "transformation.hpp"
#include <iostream>
using namespace std;

void Homogeneous_forward_transform(vector<vector<double>> before_matrix, vector<vector<double>> trans_matrix_1, vector<vector<double>>& after_matrix)
{
	int trans_cnt = 0;
	int cnt = 0;
	vector<double> tmp;
	while (cnt < SIZE)
	{
		vector<double> row(4, 0);
		for (int i = 0; i < SIZE; i++)
			tmp.push_back(trans_matrix_1.at(cnt).at(i));

		//행렬 곱셈 계산 결과의 row는 곱해지는 행렬(오른쪽 행렬)의 row들의 선형조합이므로 곱하는 행렬(왼쪽 행렬)의 원소를 오른쪽 행렬의 row에 곱해서 row부터 순서대로 계산한다.
		for (int i = 0; i < SIZE; i++)
		{
			double element = 0;
			for (int j = 0; j < SIZE; j++)
			{
				element = tmp.at(i) * before_matrix.at(i).at(j);
				row.at(j) += element;
			}
		}
		//계산 결과를 row에 저장
		after_matrix.push_back(row);
		row.clear();
		tmp.clear();
		cnt++;
	}
}

void Homogeneous_inverse_matrix(std::vector<std::vector<double>> before_matrix, std::vector<std::vector<double>>& inverse_matrix)
{
	vector<double> position_column(4, 0);
	double position = 0;
	//4x4 행렬에서 position단은 transpose하지 않고 position x noa 벡터 곱으로 표현 되므로 따로 계산 해준다.
	for (int i = 0; i < SIZE - 1; i++)
	{
		for (int j = 0; j < SIZE - 1; j++)
			position -= (before_matrix.at(j).at(SIZE - 1) * before_matrix.at(j).at(i));
		position_column.at(i) += position;
		position = 0;
	}
	//rotation 원소들은 단순 transpose시켜준다.
	for (int i = 0; i < SIZE - 1; i++)
	{
		vector<double> row;
		for (int j = 0; j < SIZE - 1; j++)
		{
			position = before_matrix.at(j).at(i);
			row.push_back(position);
		}
		//위에서 row에 rotation원소만 넣어 주었으므로 position원소를 마지막에 넣어 준다.
		row.push_back(position_column.at(i));
		inverse_matrix.push_back(row);
		row.clear();
	}
	//로봇 좌표계의 homogeneous matrix에서 마지막 row 는 0001 으로 고정되므로 마지막에 따로 추가해 준다.
	inverse_matrix.push_back(before_matrix.at(SIZE - 1));
}


void CDisplay(std::vector<std::vector<double>> matrix)
{
	std::cout << std::fixed;
	std::cout.precision(3);
	for (int i = 0; i < SIZE; i++)
	{
		for (int j = 0; j < SIZE; j++)
		{
			std::cout.setf(std::ios::right);
			std::cout << std::setw(6) << matrix.at(i).at(j) << " ";
		}
		std::cout << std::endl;
	}
}
//roll pitch yaw 변환 함수
void Homogeneous_rpy_matrix(double* position, double* rpy_angle, std::vector<std::vector<double>>& after_matrix)
{
	double roll = rpy_angle[0], pitch = rpy_angle[1], yaw = rpy_angle[2];
	double roll_rotation[4][4] = { {COS(roll),NSIN(roll),0,0},{SIN(roll),COS(roll),0,0},{0,0,1,0},{0,0,0,1} };
	double pitch_rotation[4][4] = { {COS(pitch),0,SIN(pitch),0},{0,1,0,0},{NSIN(pitch),0,COS(pitch),0},{0,0,0,1} };
	double yaw_rotation[4][4] = { {1,0,0,0},{0,COS(yaw),NSIN(yaw),0},{0,SIN(yaw),COS(yaw),0},{0,0,0,1} };
	double translation_matrix[4][4] = { {1,0,0,position[0]},{0,1,0,position[1]},{0,0,1,position[2]},{0,0,0,1} };

	vector<vector<double>> roll_matrix;
	vector<vector<double>> pitch_matrix;
	vector<vector<double>> yaw_matrix;
	vector<vector<double>> position_matrix;

	for (int i = 0; i < SIZE; i++)
	{
		std::vector<std::vector<double>> tmp(4, std::vector<double>(4));
		std::copy(roll_rotation[i], roll_rotation[i] + SIZE, tmp.at(0).begin());
		std::copy(pitch_rotation[i], pitch_rotation[i] + SIZE, tmp.at(1).begin());
		std::copy(yaw_rotation[i], yaw_rotation[i] + SIZE, tmp.at(2).begin());
		std::copy(translation_matrix[i], translation_matrix[i] + SIZE, tmp.at(3).begin());
		roll_matrix.push_back(tmp.at(0));
		pitch_matrix.push_back(tmp.at(1));
		yaw_matrix.push_back(tmp.at(2));
		position_matrix.push_back(tmp.at(3));
		tmp.clear();
	}
	cout << "\nroll : \n";
	CDisplay(roll_matrix);
	cout << "\npitch : \n";
	CDisplay(pitch_matrix);
	cout << "\nyaw : \n";
	CDisplay(yaw_matrix);

	//roll pitch yaw 순으로 회전 하는데 noa좌표계를 기준으로 한 절대변환 이므로 yaw pitch roll 순으로 현재 위치에 행렬곱셈을 해 준다.
	//yaw rotation
	Homogeneous_forward_transform(position_matrix, yaw_matrix, after_matrix);
	yaw_matrix.clear();
	//pitch rotation
	Homogeneous_forward_transform(after_matrix, pitch_matrix, yaw_matrix);
	after_matrix.clear();
	//roll rotation
	Homogeneous_forward_transform(yaw_matrix, roll_matrix, after_matrix);
}

vector<double> Homogeneous_inverse_kinematic(std::vector<std::vector<double>> robot_base_hand_matrix, double* normal, int num_of_joint)
{
	vector<double>joint_angle;
	double n_x = robot_base_hand_matrix.at(0).at(0), n_y = robot_base_hand_matrix.at(1).at(0), n_z = robot_base_hand_matrix.at(2).at(0);
	double o_x = robot_base_hand_matrix.at(0).at(1), o_y = robot_base_hand_matrix.at(1).at(1), o_z = robot_base_hand_matrix.at(2).at(1);
	double a_x = robot_base_hand_matrix.at(0).at(2), a_y = robot_base_hand_matrix.at(1).at(2), a_z = robot_base_hand_matrix.at(2).at(2);
	double p_x = robot_base_hand_matrix.at(0).at(3), p_y = robot_base_hand_matrix.at(1).at(3), p_z = robot_base_hand_matrix.at(2).at(3);
	//calculate theta_1
	double theta_1 = ATAN2(p_y, p_x);
	//calculate theta_234
	double theta_234 = ATAN2(a_z, ((COS(theta_1) * a_x) + (SIN(theta_1) * a_y)));
	//calculate COS(theta_3) , SIN(theta_3)
	double cos_theta_3 = (pow(((p_x * COS(theta_1)) + (p_y * SIN(theta_1)) - (COS(theta_234) * normal[3])), 2) + pow((p_z - (SIN(theta_234) * 0.1)), 2) - pow(normal[1], 2) - pow(normal[2], 2)) / (2 * normal[1] * normal[2]);
	double sin_theta_3 = sqrt(1 - pow(cos_theta_3, 2));
	//calculate theta_3
	double theta_3 = ATAN2(sin_theta_3, cos_theta_3);
	//calculate theta_2
	double theta_2 = ATAN2((((cos_theta_3 * normal[2]) + normal[1]) * (p_z - (SIN(theta_234) * normal[3]))) - ((sin_theta_3 * normal[2]) * (p_x * COS(theta_1)) + (p_y * SIN(theta_1)) - (normal[3] * COS(theta_234))), ((cos_theta_3 * normal[2]) + normal[1]) * ((p_x * COS(theta_1)) + (p_y * SIN(theta_1)) - (COS(theta_234) * normal[3])) + ((SIN(theta_3) * normal[2])) * (p_z - (SIN(theta_234) * normal[3])));
	//calculate theta_4
	double theta_4 = theta_234 - theta_2 - theta_3;
	//calculate theta_5
	double theta_5 = ATAN2((COS(theta_234) * ((COS(theta_1) * a_x) + (SIN(theta_1) * a_y)) + (SIN(theta_234) * a_z)), (SIN(theta_1) * a_x) - (COS(theta_1) * a_y));
	//calculate theta_6
	double theta_6 = ATAN2((NSIN(theta_234) * ((COS(theta_1) * n_x) + (SIN(theta_1) * n_y))) + (COS(theta_234) * n_z), (COS(theta_234) * o_z) + (NSIN(theta_234) * ((COS(theta_1) * o_x) + (SIN(theta_1) * o_y))));
	joint_angle.push_back(theta_1);
	joint_angle.push_back(theta_2);
	joint_angle.push_back(theta_3);
	joint_angle.push_back(theta_4);
	joint_angle.push_back(theta_5);
	joint_angle.push_back(theta_6);
	return joint_angle;
}
