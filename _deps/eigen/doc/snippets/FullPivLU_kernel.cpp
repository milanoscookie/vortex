// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
MatrixXf m = MatrixXf::Random(3, 5);
cout << "Here is the matrix m:" << endl << m << endl;
MatrixXf ker = m.fullPivLu().kernel();
cout << "Here is a matrix whose columns form a basis of the kernel of m:" << endl << ker << endl;
cout << "By definition of the kernel, m*ker is zero:" << endl << m * ker << endl;
