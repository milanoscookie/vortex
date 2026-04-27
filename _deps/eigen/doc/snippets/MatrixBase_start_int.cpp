// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
RowVector4i v = RowVector4i::Random();
cout << "Here is the vector v:" << endl << v << endl;
cout << "Here is v.head(2):" << endl << v.head(2) << endl;
v.head(2).setZero();
cout << "Now the vector v is:" << endl << v << endl;
