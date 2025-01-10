#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::Matrix3f mat;
    mat << 1, 2, 3,
           4, 5, 6,
           7, 8, 9;
    std::cout << "Matrix:\n" << mat << std::endl;
    return 0;
}