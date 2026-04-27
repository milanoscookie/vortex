// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
Matrix4i m = Matrix4i::Random();
m.row(1).setOnes();
cout << m << endl;
