# Datalink Tamariw
This in the program that takes care of transfering data between tamariw satellites and ORPE running in a separate proccess.

All RODOS topics/messages comming from the WIFI and UART links will be forwarded.

## Building instructions on RPI Zero 2
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/port/posix.cmake ..