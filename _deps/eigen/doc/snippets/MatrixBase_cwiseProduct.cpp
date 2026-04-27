// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
Matrix3i a = Matrix3i::Random(), b = Matrix3i::Random();
Matrix3i c = a.cwiseProduct(b);
cout << "a:\n" << a << "\nb:\n" << b << "\nc:\n" << c << endl;
