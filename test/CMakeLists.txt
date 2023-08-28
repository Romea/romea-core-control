find_package(GTest REQUIRED)

get_filename_component(TEST_WITH_DATA_TEST_DIR "data" ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(${PROJECT_NAME}_test_observer_cinematic_linear_tangent test_observer_cinematic_linear_tangent.cpp )
target_link_libraries(${PROJECT_NAME}_test_observer_cinematic_linear_tangent ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_observer_cinematic_linear_tangent PRIVATE -std=c++17)
add_test(test_observer_cinematic_linear_tangent ${PROJECT_NAME}_test_observer_cinematic_linear_tangent)

add_executable(${PROJECT_NAME}_test_observer_cinematic_lyapunov test_observer_cinematic_lyapunov.cpp )
target_link_libraries(${PROJECT_NAME}_test_observer_cinematic_lyapunov ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_observer_cinematic_lyapunov PRIVATE -std=c++17)
add_test(test_observer_cinematic_lyapunov ${PROJECT_NAME}_test_observer_cinematic_lyapunov)

add_executable(${PROJECT_NAME}_test_follow_trajectory_classic_sliding test_follow_trajectory_classic_sliding.cpp )
target_link_libraries(${PROJECT_NAME}_test_follow_trajectory_classic_sliding ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_follow_trajectory_classic_sliding PRIVATE -std=c++17)
add_test(test_follow_trajectory_classic_sliding ${PROJECT_NAME}_test_follow_trajectory_classic_sliding)

add_executable(${PROJECT_NAME}_test_follow_trajectory_predictive_sliding test_follow_trajectory_predictive_sliding.cpp )
target_link_libraries(${PROJECT_NAME}_test_follow_trajectory_predictive_sliding ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_follow_trajectory_predictive_sliding PRIVATE -std=c++17)
add_test(test_follow_trajectory_predictive_sliding ${PROJECT_NAME}_test_follow_trajectory_predictive_sliding)

add_executable(${PROJECT_NAME}_test_follow_me_skid_steering test_follow_me_skid_steering.cpp )
target_link_libraries(${PROJECT_NAME}_test_follow_me_skid_steering ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_follow_me_skid_steering PRIVATE -std=c++17)
add_test(test_follow_me_skid_steering ${PROJECT_NAME}_test_follow_me_skid_steering)

add_executable(${PROJECT_NAME}_test_follow_me_two_axle_steering test_follow_me_two_axle_steering.cpp )
target_link_libraries(${PROJECT_NAME}_test_follow_me_two_axle_steering ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_follow_me_two_axle_steering PRIVATE -std=c++17)
add_test(test_follow_me_two_axle_steering ${PROJECT_NAME}_test_follow_me_two_axle_steering)
