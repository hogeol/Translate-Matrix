#include <vector>
#ifndef __TRANSFORMATION__
#define __TRANSFORMATION__

#define SIZE 4

void Homogeneous_forward_transform(std::vector<std::vector<double>> before_matrix, std::vector<std::vector<double>> trans_matrix, std::vector<std::vector<double>>& after_matrix);
void Homogeneous_inverse_matrix(std::vector<std::vector<double>> before_matrix, std::vector<std::vector<double>>& inverse_matrix);

#endif __TRANSFORMATION__
