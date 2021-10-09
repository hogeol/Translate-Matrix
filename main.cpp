#include <iostream>
#include <vector>
#include "transformation.hpp"
#include "gui.hpp"
using namespace std;

int main(int argc, char** argv)
{
	vector<vector<double>> uTh; 
	vector<vector<double>> uTr;
	vector<vector<double>> rTu;
	vector<vector<double>> rTh;
	
	//�־��� ���
	double u_hand_element_array[SIZE][SIZE] = { {1,0,0,0},{0,1,0,6},{0,0,1,2},{0,0,0,1} };
	double u_robot_element_array[SIZE][SIZE] = { {0,0,-1,8},{0,1,0,2},{1,0,0,-1},{0,0,0,1} };
	
	//2���� �迭 -> ���� ��ȯ
	Input(u_hand_element_array, uTh);
	Input(u_robot_element_array, uTr);
	
	//����� ��ȯ
	Homogeneous_inverse_matrix(uTr, rTu);
	
	//��� �������
	Homogeneous_forward_transform(uTh, rTu, rTh);

	//���
	cout << "universe base Hand matrix\n\n";
	Display(uTh);
	cout << "\nuniverse base Robot matrix\n\n";
	Display(uTr);
	cout << "\nuniverse base Robot inverse-matrix\n\n";
	Display(rTu);
	cout << "\nRobot base Hand matrix\n\n";
	Display(rTh);
	
	return 0;
}