// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
MatrixXcf A = MatrixXcf::Random(4, 4);
cout << "Here is a random 4x4 matrix, A:" << endl << A << endl << endl;
ComplexSchur<MatrixXcf> schurOfA(A, false);  // false means do not compute U
cout << "The triangular matrix T is:" << endl << schurOfA.matrixT() << endl;
