官方文档:http://eigen.tuxfamily.org
矩阵部分文档:https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html
## 转置/共轭/伴随
```cpp
MatrixXcf a = MatrixXcf::Random(2,2);
cout << "Here is the matrix a\n" << a << endl;

// 转置: 将矩阵的行和列对调得到的新矩阵。
cout << "Here is the matrix a^T\n" << a.transpose() << endl;
// 共轭: 将矩阵中每个复数的虚部取相反数得到的新矩阵。
cout << "Here is the conjugate of a\n" << a.conjugate() << endl;
// 伴随: 转置共轭
cout << "Here is the matrix a^*\n" << a.adjoint() << endl;

// 不要做a = a.transpose() / a = a.adjoint(), 因为实际上在赋值时才进行转置/伴随
// 但是可以用a = a.transposeInPlace()和a = a.adjointInPlace()完成替换操作
```
output: 
```
Here is the initial matrix a:
1 2 3
4 5 6
and after being transposed:
1 4
2 5
3 6
```

## 矩阵乘法
```cpp
#include <iostream>
#include <Eigen/Dense>
 
int main()
{
  Eigen::Matrix2d mat;
  mat << 1, 2,
         3, 4;
  /*
  [1 2;
  3 4]
  */
  Eigen::Vector2d u(-1,1), v(2,0);
  // 矩阵*矩阵
  std::cout << "Here is mat*mat:\n" << mat*mat << std::endl;
  // 矩阵*向量
  std::cout << "Here is mat*u:\n" << mat*u << std::endl;
  std::cout << "Here is u^T*mat:\n" << u.transpose()*mat << std::endl;
  // 向量*向量
  std::cout << "Here is u^T*v:\n" << u.transpose()*v << std::endl;
  std::cout << "Here is u*v^T:\n" << u*v.transpose() << std::endl;
  std::cout << "Let's multiply mat by itself" << std::endl;
  mat = mat*mat;
  std::cout << "Now mat is mat:\n" << mat << std::endl;
}
```
output:
```
Here is mat*mat:
 7 10
15 22
Here is mat*u:
1
1
Here is u^T*mat:
2 2
Here is u^T*v:
-2
Here is u*v^T:
-2 -0
 2  0
Let's multiply mat by itself
Now mat is mat:
 7 10
15 22
```

## 点乘/叉乘
```cpp
#include <iostream>
#include <Eigen/Dense>
 
int main()
{
  Eigen::Vector3d v(1,2,3);
  Eigen::Vector3d w(0,1,2);
 
  // 点乘, 可以直接用dot函数, 也可以转置后再乘
  std::cout << "Dot product: " << v.dot(w) << std::endl;
  double dp = v.adjoint()*w; // automatic conversion of the inner product to a scalar
  std::cout << "Dot product via a matrix product: " << dp << std::endl;

  // 叉乘(only for vectors of size 3)
  std::cout << "Cross product:\n" << v.cross(w) << std::endl;
}
```
output:
```
Dot product: 8
Dot product via a matrix product: 8
Cross product:
 1
-2
 1
```
注: 在使用复数(complex number)时，Eigen的点积在第一个变量中是共轭线性的，在第二个变量中是线性的。

所谓共轭线性，指的是对于复数 $z$ 和 $w$，以及实数 $a$，有 $z \cdot (aw) = a(z \cdot w)$。

例如，在 Eigen 中，如果 $x$ 和 $y$ 是复数向量，则点积的定义为：

$$x \cdot y = \sum_{i=1}^n x_i y_i^*$$

其中 $y_i^*$ 表示 $y_i$ 的共轭复数（即 $y_i$ 的实部和虚部相反）。

从上面的定义可以看出，对于复数 $z$ 和 $w$，以及实数 $a$，有 $z \cdot (aw) = a(z \cdot w)$，即点积是共轭线性的。

## 基础数学运算
```cpp
#include <iostream>
#include <Eigen/Dense>
 
using namespace std;
int main()
{
  // Coeff = Coefficients, 系数
  // trace = 对角线之和, a.trace() = a.diagnol().sum()
  Eigen::Matrix2d mat;
  mat << 1, 2,
         3, 4;
  cout << "Here is mat.sum():       " << mat.sum()       << endl;
  cout << "Here is mat.prod():      " << mat.prod()      << endl;
  cout << "Here is mat.mean():      " << mat.mean()      << endl;
  cout << "Here is mat.minCoeff():  " << mat.minCoeff()  << endl;
  cout << "Here is mat.maxCoeff():  " << mat.maxCoeff()  << endl;
  cout << "Here is mat.trace():     " << mat.trace()     << endl;
}
```
output:
```
Here is mat.sum():       10
Here is mat.prod():      24
Here is mat.mean():      2.5
Here is mat.minCoeff():  1
Here is mat.maxCoeff():  4
Here is mat.trace():     5
```

可以给一些函数传参以获得minCoeff或maxCoeff元素所在的位置:
```cpp
  Matrix3f m = Matrix3f::Random();
  std::ptrdiff_t i, j; // 位置类型参数
  float minOfM = m.minCoeff(&i,&j);
  cout << "Here is the matrix m:\n" << m << endl;
  cout << "Its minimum coefficient (" << minOfM 
       << ") is at position (" << i << "," << j << ")\n\n";
 
  RowVector4i v = RowVector4i::Random();
  int maxOfV = v.maxCoeff(&i);
  cout << "Here is the vector v: " << v << endl;
  cout << "Its maximum coefficient (" << maxOfV 
       << ") is at position " << i << endl;
```
output:
```
Here is the matrix m:
  0.68  0.597  -0.33
-0.211  0.823  0.536
 0.566 -0.605 -0.444
Its minimum coefficient (-0.605) is at position (2,1)

Here is the vector v:  1  0  3 -3
Its maximum coefficient (3) is at position 2
```
## 检查
一种是编译时检查, 会用大写字母突出显示错误, 比如:
```cpp
Matrix3f m;
Vector4f v;
v = m*v;      // Compile-time error: YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES
```
另一种是动态检查, 会在运行时通过assertions判断操作是否合法, 比如:
```cpp
MatrixXf m(3,3);
VectorXf v(4);
v = m * v; // Run-time assertion failure here: "invalid matrix product"
```