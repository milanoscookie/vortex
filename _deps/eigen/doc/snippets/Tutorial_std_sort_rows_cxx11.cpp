// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
ArrayXXi A = ArrayXXi::Random(4, 4).abs();
cout << "Here is the initial matrix A:\n" << A << "\n";
for (auto row : A.rowwise()) std::sort(row.begin(), row.end());
cout << "Here is the sorted matrix A:\n" << A << "\n";
