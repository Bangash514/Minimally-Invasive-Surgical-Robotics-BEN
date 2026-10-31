cmake_minimum_required(VERSION 3.10)
project(SurgicalRoboticsCpp CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(PROGRAMS
    transforms_chain two_link_arm rotations_basics trajectory_cubic
    stewart_platform coordinated_motion workspace_singularities
    hand_eye_calibration force_control state_machine
    segmentation_metrics validation_metrics)

foreach(prog ${PROGRAMS})
  add_executable(${prog} ${prog}.cpp)
  target_include_directories(${prog} PRIVATE include)
  target_link_libraries(${prog} Eigen3::Eigen)
endforeach()
