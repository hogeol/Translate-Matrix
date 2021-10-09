#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#ifndef __GUI__
#define __GUI__

void Input(double element[][4], std::vector<std::vector<double>>& matrix);
void Display(std::vector<std::vector<double>> matrix);

void Input(double element[][4], std::vector<std::vector<double>>& matrix)
{
	int cnt = 0;
	int n = SIZE;

	for (int i = 0; i < SIZE; i++)
	{
		std::vector<double> tmp(4);
		copy(element[i], element[i] + n, tmp.begin());
		matrix.push_back(tmp);
		tmp.clear();
	}
}

void Display(std::vector<std::vector<double>> matrix)
{
	for (int i = 0; i < SIZE; i++)
	{
		for (int j = 0; j < SIZE; j++)
		{
			std::cout.setf(std::ios::right);
			std::cout << std::setw(3) << matrix.at(i).at(j) << " ";
		}
		std::cout << std::endl;
	}
}

#endif __GUI__
