#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
#include<vector>
#include<array>
#include<map>

#define PI acos(-1)

using namespace std;
using namespace Eigen;


int main() {

	// Basic Example of cpp
	std::cout << "Example of cpp \n";
	float a = 1.0, b = 2.0;
	std::cout << a << std::endl;
	std::cout << a / b << std::endl;
	std::cout << std::sqrt(b) << std::endl;
	std::cout << std::acos(-1) << std::endl;
	std::cout << std::sin(30.0 / 180.0*acos(-1)) << std::endl << endl;

	// Example of vector
	std::cout << "Example of vector \n";
	// vector definition
	Eigen::Vector3f v(1.0f, 2.0f, 3.0f);
	Eigen::Vector3f w(1.0f, 0.0f, 0.0f);
	// vector output
	std::cout << "Example of output \n";
	std::cout << v << std::endl;
	// vector add
	std::cout << "Example of add \n";
	std::cout << v + w << std::endl;
	// vector scalar multiply
	std::cout << "Example of scalar multiply \n";
	std::cout << v * 3.0f << std::endl;
	std::cout << 2.0f * v << std::endl << endl;

	// Example of matrix
	std::cout << "Example of matrix \n";
	// matrix definition
	Eigen::Matrix3f i, j;
	i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
	j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
	// matrix output
	std::cout << "Example of output \n";
	std::cout << i << std::endl;
	// matrix add i + j
	cout << i + j << endl;
	// matrix scalar multiply i * 2.0
	cout << 2.f * i << endl;
	// matrix multiply i * j
	cout << j * j << endl;
	// matrix multiply vector i * v
	cout << i * v << endl << endl;

	// Assignment 0
	Matrix3f m_affine;
	float ang = (float)(PI / 4.);
	m_affine << cos(ang), -sin(ang), 1., sin(ang), cos(ang), 2., 0., 0., 1.;
	Vector3f point(2., 1., 1.);
	cout << "result:\n";
	cout << m_affine * point << endl;

	return 0;
}