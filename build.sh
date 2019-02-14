#!cd /bin/sh
rm -rf build
mkdir build
cd build
cmake .. && make
./path_planning
