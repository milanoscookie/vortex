// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
Matrix2cf m = Matrix2cf::Random();
cout << "Here is the 2x2 complex matrix m:" << endl << m << endl;
cout << "Here is the adjoint of m:" << endl << m.adjoint() << endl;
