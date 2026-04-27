// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
MatrixXd ones = MatrixXd::Ones(3, 3);
VectorXd eivals = ones.selfadjointView<Lower>().eigenvalues();
cout << "The eigenvalues of the 3x3 matrix of ones are:" << endl << eivals << endl;
