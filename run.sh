cmake -S . -B build -DCMAKE_CXX_FLAGS='-Wall -Wextra' && cmake --build build -j4 &&
  echo Running RadarDemo &&
  time ./build/src/RadarDemo &&
  echo Running Analyze &&
  time python ./tools/fmcw_car_tracker.py
