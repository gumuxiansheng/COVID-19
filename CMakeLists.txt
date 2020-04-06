cmake_minimum_required(VERSION 3.0.0)
project(Demo VERSION 0.1.0)

include(CTest)
enable_testing()
if (APPLE)
	set(CMAKE_INSTALL_RPATH
		"@loader_path/../..;@loader_path/../lib;@loader_path")
else()
	set(CMAKE_INSTALL_RPATH "$ORIGIN/../../:$ORIGIN/../lib:$ORIGIN/")
endif()

include_directories("/usr/local/include")
link_directories("/usr/local/lib")

get_filename_component(PARENT_SOURCE_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)

include(GNUInstallDirs)

find_package(ortools)

foreach(EXECUTABLE
  hello)
  add_executable(${EXECUTABLE} ${EXECUTABLE}.cpp)
  target_include_directories(${EXECUTABLE} PUBLIC ${PARENT_SOURCE_DIR})
  target_compile_features(${EXECUTABLE} PRIVATE cxx_std_11)
  target_link_libraries(${EXECUTABLE} PRIVATE ortools::ortools)
  install(TARGETS ${EXECUTABLE}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endforeach()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
