// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
int array[9];
for (int i = 0; i < 9; ++i) array[i] = i;
cout << Map<Matrix3i>(array) << endl;
