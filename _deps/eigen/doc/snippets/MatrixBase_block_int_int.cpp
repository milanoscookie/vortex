// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
Matrix4i m = Matrix4i::Random();
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Here is m.block<2,2>(1,1):" << endl << m.block<2, 2>(1, 1) << endl;
m.block<2, 2>(1, 1).setZero();
cout << "Now the matrix m is:" << endl << m << endl;
