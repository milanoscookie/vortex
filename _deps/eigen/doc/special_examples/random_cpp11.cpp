// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
#include <Eigen/Core>
#include <iostream>
#include <random>

int main() {
  std::default_random_engine generator;
  std::poisson_distribution<int> distribution(4.1);
  auto poisson = [&]() { return distribution(generator); };

  Eigen::RowVectorXi v = Eigen::RowVectorXi::NullaryExpr(10, poisson);
  std::cout << v << "\n";
}
