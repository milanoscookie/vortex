// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
MatrixXi m = MatrixXi::Random(2, 3);
cout << "Here is the matrix m:" << endl << m << endl;
cout << "m.colwise().replicate<3>() = ..." << endl;
cout << m.colwise().replicate<3>() << endl;
