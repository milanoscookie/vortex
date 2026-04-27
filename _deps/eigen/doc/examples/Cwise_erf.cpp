// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
#include <Eigen/Core>
#include <unsupported/Eigen/SpecialFunctions>
#include <iostream>
int main() {
  Eigen::Array4d v(-0.5, 2, 0, -7);
  std::cout << v.erf() << std::endl;
}
