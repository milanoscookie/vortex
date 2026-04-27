// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
Matrix4i m = Matrix4i::Zero();
m.block<3, 3>(1, 0).setIdentity();
cout << m << endl;
