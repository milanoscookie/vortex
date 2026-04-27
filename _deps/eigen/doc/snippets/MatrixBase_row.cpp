// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
Matrix3d m = Matrix3d::Identity();
m.row(1) = Vector3d(4, 5, 6);
cout << m << endl;
