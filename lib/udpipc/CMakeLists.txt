cmake_minimum_required(VERSION 3.5.0)
project(udpipc VERSION 0.1.0)

#Collect and add source files
file(GLOB_RECURSE SRC_FILES src/*.cpp)

add_library(${PROJECT_NAME} ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC include/)