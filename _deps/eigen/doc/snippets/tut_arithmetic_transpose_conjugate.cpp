// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
MatrixXcf a = MatrixXcf::Random(2, 2);
cout << "Here is the matrix a\n" << a << endl;

cout << "Here is the matrix a^T\n" << a.transpose() << endl;

cout << "Here is the conjugate of a\n" << a.conjugate() << endl;

cout << "Here is the matrix a^*\n" << a.adjoint() << endl;
