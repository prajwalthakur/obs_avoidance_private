cmake_minimum_required(VERSION 3.14)
project_add_library()
# Optional: Eigen is already a dependency of pinocchio target, but finding it explicitly is harmless
find_package(Eigen3 QUIET)
project_add_subdirectories()