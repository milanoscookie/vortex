// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
Matrix3f m = Matrix3f::Random();
Matrix3f y = Matrix3f::Random();
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Here is the matrix y:" << endl << y << endl;
Matrix3f x;
x = m.colPivHouseholderQr().solve(y);
assert(y.isApprox(m* x));
cout << "Here is a solution x to the equation mx=y:" << endl << x << endl;
