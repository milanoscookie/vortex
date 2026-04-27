// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
MatrixXf matA(2, 2);
matA << 2, 0, 0, 2;
matA = matA * matA;
cout << matA;
