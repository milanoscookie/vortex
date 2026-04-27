// Vendored Eigen tutorial sources demonstrate library usage patterns and examples. @feature:eigen-docs
Array33i a = Array33i::Random(), b = Array33i::Random();
Array33i c = a * b;
cout << "a:\n" << a << "\nb:\n" << b << "\nc:\n" << c << endl;
