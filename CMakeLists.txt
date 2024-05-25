cmake_minimum_required(VERSION 3.5.0)
project(Datalink VERSION 0.1.0)

#add RODOS
add_subdirectory(lib/rodos)

#add the udpipc library for communication with ORPE
add_subdirectory(lib/udpipc)

#add orpe to use the data structs. Do not link as library.
add_subdirectory(lib/ORPE)

#Collect datalink source files add them to the executable
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})

#Create the datalink executable
target_link_libraries(${PROJECT_NAME} PUBLIC rodos udpipc)

