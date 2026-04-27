// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
MatrixXcf v = MatrixXcf::Random(2, 3);
cout << v << endl << endl;
cout << v.cwiseArg() << endl;