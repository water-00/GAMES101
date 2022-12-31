#include <eigen3/Eigen/Eigen>
#include <iostream>

int main() {
    Eigen::Matrix<float, 4, 1> s(1.0, 2.0, 3.0, 4.0);
    std::cout << s << std::endl;
    for (int i = 0; i < s.size(); i++)
        std::cout << s[i] << std::endl;
}