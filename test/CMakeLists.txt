get_filename_component(TEST_WITH_DATA_TEST_DIR "data" ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

catkin_add_gtest(${PROJECT_NAME}_test_observer_cinematic_linear_tangent test_observer_cinematic_linear_tangent.cpp )
target_link_libraries(${PROJECT_NAME}_test_observer_cinematic_linear_tangent ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}_test_observer_cinematic_lyapounov test_observer_cinematic_lyapounov.cpp )
target_link_libraries(${PROJECT_NAME}_test_observer_cinematic_lyapounov ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}_test_follow_trajectory_classic_sliding test_follow_trajectory_classic_sliding.cpp )
target_link_libraries(${PROJECT_NAME}_test_follow_trajectory_classic_sliding ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}_test_follow_trajectory_predictive_sliding test_follow_trajectory_predictive_sliding.cpp )
target_link_libraries(${PROJECT_NAME}_test_follow_trajectory_predictive_sliding ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}_test_follow_me_skid_steering test_follow_me_skid_steering.cpp )
target_link_libraries(${PROJECT_NAME}_test_follow_me_skid_steering ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}_test_follow_me_two_axle_steering test_follow_me_two_axle_steering.cpp )
target_link_libraries(${PROJECT_NAME}_test_follow_me_two_axle_steering ${PROJECT_NAME})

