cmake_minimum_required(VERSION 3.0.0)
set(PROJECT_NAME HELLO)
project(${PROJECT_NAME} VERSION 0.1.0)

add_executable(${PROJECT_NAME} src/hello.cc)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)

add_subdirectory(src)
