cmake_minimum_required(VERSION 3.14)

project(omESPTests LANGUAGES CXX)

include(../cmake/project-is-top-level.cmake)
include(../cmake/folders.cmake)
include(../cmake/windows-set-path.cmake)

if(PROJECT_IS_TOP_LEVEL)
  find_package(omESP REQUIRED)
  enable_testing()
endif()

add_executable(omESP_test source/omESP_test.cpp)
target_link_libraries(omESP_test PRIVATE omESP::omESP)
target_compile_features(omESP_test PRIVATE cxx_std_17)

add_test(NAME omESP_test COMMAND omESP_test)
windows_set_path(omESP_test omESP::omESP)

add_folders(Test)
