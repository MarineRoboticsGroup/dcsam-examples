add_executable(icp icp.cpp)
target_link_libraries(icp dcsam gtsam)

add_executable(robust_pgo robust_pgo.cpp)
target_link_libraries(robust_pgo dcsam gtsam)

add_executable(robust_pgo_mc robust_pgo_mc.cpp)
target_link_libraries(robust_pgo_mc dcsam gtsam)
