#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

#define PI acos(-1)

// 给定一个点 P = (2,1), 将该点绕原点先逆时针旋转 45◦ ，再平移 (1,2), 
// 计算出变换后点的坐标（要求用齐次坐标进行计算）。

int main() {
    Eigen::Vector3d P(2, 1, 1);
    Eigen::Matrix3d Linear_Matrix;
    Linear_Matrix << cos(PI/4), -sin(PI/4), 1, 
                    sin(PI/4), cos(PI/4), 2,
                    0, 0, 1;
    Eigen::Vector3d P_prime;
    P_prime = Linear_Matrix * P;
    std::cout << P_prime << std::endl;
}