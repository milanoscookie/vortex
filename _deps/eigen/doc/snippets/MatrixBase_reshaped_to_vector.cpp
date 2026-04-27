// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
Matrix4i m = Matrix4i::Random();
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Here is m.reshaped().transpose():" << endl << m.reshaped().transpose() << endl;
cout << "Here is m.reshaped<RowMajor>().transpose():  " << endl << m.reshaped<RowMajor>().transpose() << endl;
