cmake_minimum_required(VERSION "3.9.0")

project("cmaze" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD "17")
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

add_executable("cmaze"
    "src/cmaze.cpp"
)
target_include_directories("cmaze" PRIVATE
    "include"
)