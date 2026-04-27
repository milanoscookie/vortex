// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
MatrixXd m(2, 3);
m << 2, -4, 6, -5, 1, 0;
cout << m.cwiseAbs2() << endl;
