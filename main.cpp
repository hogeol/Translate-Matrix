#include <iostream>
#include "transformation.hpp"
#include "gui.hpp"
using namespace std;

int main(int argc, char** argv)
{
	vector<vector<double>> uTh;
	vector<vector<double>> uTr;
	vector<vector<double>> rTu;
	vector<vector<double>> rTh;
	vector<vector<double>> rpy_result;
	vector<double> joint_result;
	//주어진 행렬
	double u_hand_element_array[SIZE][SIZE] = { {1,0,0,0},{0,1,0,6},{0,0,1,2},{0,0,0,1} };
	double u_robot_element_array[SIZE][SIZE] = { {0,0,-1,8},{0,1,0,2},{1,0,0,-1},{0,0,0,1} };
	double pos[3] = { 1.5,1.0,0.5 };
	double rpy[3] = { 90,45,30 };
	double normal_an[JOINT] = { 0,1,1,0.1,0,0 };
	/*
	//Exercise 1
	//2차원 배열 -> 벡터 변환
	Input(u_hand_element_array, uTh);
	Input(u_robot_element_array, uTr);

	//역행렬 변환
	Homogeneous_inverse_matrix(uTr, rTu);

	//행렬 곱셈계산
	Homogeneous_forward_transform(uTh, rTu, rTh);

	//출력
	cout << "universe base Hand matrix\n\n";
	Display(uTh);
	cout << "\nuniverse base Robot matrix\n\n";
	Display(uTr);
	cout << "\nuniverse base Robot inverse-matrix\n\n";
	Display(rTu);
	cout << "\nRobot base Hand matrix\n\n";
	Display(rTh);
	*/

	/*
	//Exercise 2
	cout << "roll : " << rpy[0] << ", pitch : " << rpy[1] << ", yaw : " << rpy[2] << endl;
	printf("\nposition : %.2f, %.2f, %.2f \n", pos[0], pos[1], pos[2]);
	
	//roll, pitch, yaw 변환
	Homogeneous_rpy_matrix(pos, rpy, rpy_result);

	//rpy 변환 후 matrix 출력
	cout << "\nAfter roll pitch yaw transform:\n";
	Display(rpy_result);

	//joint angle 계산
	joint_result = Homogeneous_inverse_kinematic(rpy_result, normal_an, JOINT);

	//joint angle 출력
	for (int i = 0; i < JOINT; i++)
	{
		cout << "\njoint angle(theta) " << i + 1 << " : " << joint_result.at(i) << endl;
	}
	*/
	return 0;
}
