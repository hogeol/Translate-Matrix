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
		
		//��� ���� ��� ����� row�� �������� ���(������ ���)�� row���� ���������̹Ƿ� ���ϴ� ���(���� ���)�� ���Ҹ� ������ ����� row�� ���ؼ� row���� ������� ����Ѵ�.
		for (int i = 0; i < SIZE; i++)
		{
			double element = 0;
			for (int j = 0; j < SIZE; j++)
			{
				element = tmp.at(i) * before_matrix.at(i).at(j);
				row.at(j) += element;
			}
		}
		//��� ����� row�� ����
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
	//4x4 ��Ŀ��� position���� transpose���� �ʰ� position x noa ���� ������ ǥ�� �ǹǷ� ���� ��� ���ش�.
	for (int i = 0; i < SIZE - 1; i++)
	{
		for (int j = 0; j < SIZE - 1; j++)
			position -= (before_matrix.at(j).at(SIZE - 1) * before_matrix.at(j).at(i));
		position_column.at(i) += position;
		position = 0;
	}
	//rotation ���ҵ��� �ܼ� transpose�����ش�.
	for (int i = 0; i < SIZE - 1; i++)
	{
		vector<double> row;
		for (int j = 0; j < SIZE - 1; j++)
		{
			position = before_matrix.at(j).at(i);
			row.push_back(position);
		}
		//������ row�� rotation���Ҹ� �־� �־����Ƿ� position���Ҹ� �������� �־� �ش�.
		row.push_back(position_column.at(i));
		inverse_matrix.push_back(row);
		row.clear();
	}
	//�κ� ��ǥ���� homogeneous matrix���� ������ row �� 0001 ���� �����ǹǷ� �������� ���� �߰��� �ش�.
	inverse_matrix.push_back(before_matrix.at(SIZE - 1));
}
