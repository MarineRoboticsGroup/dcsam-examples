cmake_minimum_required(VERSION 3.15)
project(dcsam_examples)

# Set some compilation options.
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wpedantic -Wextra)

# External package dependencies.
find_package(GTSAM 4.2 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(dcsam REQUIRED)

include_directories(include)
add_subdirectory(examples)
