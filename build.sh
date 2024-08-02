mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../lib/rodos/cmake/port/posix.cmake ..
make -j 2