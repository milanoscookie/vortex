// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
#include <Eigen/Core>
#include <unsupported/Eigen/SpecialFunctions>
#include <iostream>
int main() {
  Eigen::Array4d v(0.5, 10, 0, -1);
  std::cout << v.lgamma() << std::endl;
}
