cmake_minimum_required(VERSION 3.5)

project(
  "romea_core_control"
  VERSION 0.1.0
  #  DESCRIPTION
  #  HOMEPAGE_URL
  LANGUAGES CXX)

find_package(romea_core_common REQUIRED)

find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )

add_library(${PROJECT_NAME} SHARED
  src/observer/SlidingObserverCinematic.cpp
  src/observer/SlidingObserverCinematicLinearTangent.cpp
  src/observer/SlidingObserverCinematicLyapunov.cpp
  src/observer/SpeedObserver.cpp
  src/observer/SpeedObserverRoland.cpp
  src/observer/SpeedObserverMatt.cpp
  src/command/FollowMe.cpp
  src/command/FollowTrajectoryBackStepping.cpp
  src/command/FollowTrajectoryClassicSliding.cpp
  src/command/FollowTrajectoryPredictiveSliding.cpp
  src/command/KeepInterdistance.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)


target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${romea_core_common_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -O3 -std=c++17)

target_link_libraries(${PROJECT_NAME} PUBLIC
  romea_core_common::romea_core_common)

include(GNUInstallDirs)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" INSTALL_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

