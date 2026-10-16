cmake_minimum_required(VERSION 3.18)
project(terrain_path_planner VERSION 0.1.0 LANGUAGES CXX)
if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(FATAL_ERROR "Use an out-of-source build, for example: cmake -S cpp -B build/cpp")
endif()

include(CTest)
include(GNUInstallDirs)
include(FetchContent)
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp REQUIRED)
find_path(LBFGSPP_INCLUDE_DIR LBFGSB.h)
if(NOT LBFGSPP_INCLUDE_DIR)
  FetchContent_Declare(lbfgspp
    URL https://codeload.github.com/yixuan/LBFGSpp/tar.gz/refs/tags/v0.3.0
    URL_HASH SHA256=490720b9d5acce6459cb0336ca3ae0ffc48677225f0ebfb35c9bef6baefdfc6a
    SOURCE_SUBDIR include)
  FetchContent_MakeAvailable(lbfgspp)
  set(LBFGSPP_INCLUDE_DIR "${lbfgspp_SOURCE_DIR}/include")
endif()

add_library(terrain_planner src/grid.cpp src/graph.cpp src/optimizer.cpp)
add_library(terrain_planner::terrain_planner ALIAS terrain_planner)
target_compile_features(terrain_planner PUBLIC cxx_std_17)
target_include_directories(terrain_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_include_directories(terrain_planner SYSTEM PRIVATE "${LBFGSPP_INCLUDE_DIR}")
target_link_libraries(terrain_planner PRIVATE Eigen3::Eigen)
set_target_properties(terrain_planner PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(terrain_planner_json src/json.cpp)
target_link_libraries(terrain_planner_json PUBLIC terrain_planner)
target_include_directories(terrain_planner_json SYSTEM PUBLIC
  $<BUILD_INTERFACE:${NLOHMANN_JSON_INCLUDE_DIR}>)
add_executable(terrain-plan src/main.cpp)
target_link_libraries(terrain-plan PRIVATE terrain_planner_json)
add_executable(terrain-plan-example examples/plan_route.cpp)
target_link_libraries(terrain-plan-example PRIVATE terrain_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(terrain_planner PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(terrain_planner_json PRIVATE -Wall -Wextra -Wpedantic)
endif()
if(BUILD_TESTING)
  add_executable(terrain-planner-tests tests/test_planner.cpp)
  target_link_libraries(terrain-planner-tests PRIVATE terrain_planner_json)
  add_test(NAME native-regressions COMMAND terrain-planner-tests)
  add_test(NAME native-example COMMAND terrain-plan-example)
endif()

install(TARGETS terrain_planner terrain_planner_json terrain-plan
  EXPORT terrain_plannerTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ../LICENSE ../NOTICE.md ../NOTICE.en.md
  DESTINATION ${CMAKE_INSTALL_DATADIR}/terrain_path_planner)
install(FILES third_party/NOTICE.txt
  DESTINATION ${CMAKE_INSTALL_DATADIR}/terrain_path_planner/cpp/third_party)
install(EXPORT terrain_plannerTargets NAMESPACE terrain_planner::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/terrain_planner)
include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/terrain_plannerConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/terrain_plannerConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/terrain_planner)
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/terrain_plannerConfigVersion.cmake"
  COMPATIBILITY SameMajorVersion)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/terrain_plannerConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/terrain_plannerConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/terrain_planner)
