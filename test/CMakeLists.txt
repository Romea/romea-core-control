find_package(GTest REQUIRED)

get_filename_component(TEST_WITH_DATA_TEST_DIR "data" ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_compile_options(-std=c++17)

# name of the unit test to generate. It corresponds to name of the cpp file without extension
set(test_names 
  observer_cinematic_linear_tangent
  observer_cinematic_lyapunov
  follow_trajectory_classic_sliding
  follow_trajectory_predictive_sliding
  follow_me_skid_steering
  follow_me_two_axle_steering
  follow_trajectory_skid_steering_sign
)

foreach(test_name ${test_names})
  string(REPLACE " " "_" target_name "${PROJECT_NAME}_test_${test_name}")
  add_executable(${target_name} test_${test_name}.cpp)
  target_link_libraries(${target_name} ${PROJECT_NAME} GTest::GTest GTest::Main)
  add_test(${target_name} ${target_name})
endforeach()
