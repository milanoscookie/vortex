// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
Matrix3d m = Matrix3d::Random();
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Here is the sum of each row:" << endl << m.rowwise().sum() << endl;
cout << "Here is the maximum absolute value of each row:" << endl << m.cwiseAbs().rowwise().maxCoeff() << endl;
