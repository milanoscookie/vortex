// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
MatrixXcf ones = MatrixXcf::Ones(3, 3);
ComplexEigenSolver<MatrixXcf> ces(ones, /* computeEigenvectors = */ false);
cout << "The eigenvalues of the 3x3 matrix of ones are:" << endl << ces.eigenvalues() << endl;
