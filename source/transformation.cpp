#include "transformation.hpp"

using namespace std;

void Homogeneous_forward_transform(vector<vector<double>> before_matrix, vector<vector<double>> trans_matrix, vector<vector<double>>& after_matrix)
{
	int trans_cnt = 0;
	int cnt = 0;
	vector<double> tmp;
	while (cnt < SIZE)
	{
		vector<double> row(4, 0);
		for (int i = 0; i < SIZE; i++)
			tmp.push_back(trans_matrix.at(cnt).at(i));
		
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
