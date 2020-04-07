cmake_minimum_required(VERSION 3.0.0)
project(Hello VERSION 0.1.0)

find_package(ortools CONFIG REQUIRED)

add_executable(Hello hello.cpp)
target_compile_features(Hello PRIVATE cxx_std_11)
# target_include_directories(Hello PUBLIC "/usr/local/include")
target_link_libraries(Hello PUBLIC ortools::ortools)
