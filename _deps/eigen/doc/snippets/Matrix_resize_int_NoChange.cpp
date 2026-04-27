// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
MatrixXd m(3, 4);
m.resize(5, NoChange);
cout << "m: " << m.rows() << " rows, " << m.cols() << " cols" << endl;
