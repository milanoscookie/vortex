// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
MatrixXcf ones = MatrixXcf::Ones(3, 3);
ComplexEigenSolver<MatrixXcf> ces(ones);
cout << "The first eigenvector of the 3x3 matrix of ones is:" << endl << ces.eigenvectors().col(0) << endl;
