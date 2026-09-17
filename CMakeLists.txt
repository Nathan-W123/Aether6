cmake_minimum_required(VERSION 3.16)
project(aether6
        VERSION 1.0.0
        DESCRIPTION "Six-degree-of-freedom fixed-wing flight dynamics, guidance, navigation and control simulator"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(AETHER_BUILD_TESTS "Build the Catch2 unit-test suite" ON)
option(AETHER_ENABLE_WARNINGS "Enable a strict warning set" ON)
option(AETHER_ENABLE_WERROR "Treat warnings as errors" OFF)

include(FetchContent)

# ---------------------------------------------------------------------------------------
# Dependencies. Each one is taken from the system when available and fetched otherwise, so
# the project configures both on a prepared machine and on a bare CI runner with network.
# ---------------------------------------------------------------------------------------
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  message(STATUS "Eigen3 not found on the system - fetching it")
  FetchContent_Declare(Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE)
  set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(Eigen3)
endif()

find_package(yaml-cpp 0.6 QUIET)
if(NOT yaml-cpp_FOUND AND NOT TARGET yaml-cpp::yaml-cpp)
  message(STATUS "yaml-cpp not found on the system - fetching it")
  FetchContent_Declare(yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG 0.8.0
    GIT_SHALLOW TRUE)
  set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(yaml-cpp)
endif()

# Normalise the yaml-cpp target name across packaged versions.
if(TARGET yaml-cpp::yaml-cpp)
  set(AETHER_YAML_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(AETHER_YAML_TARGET yaml-cpp)
else()
  message(FATAL_ERROR "yaml-cpp target not available")
endif()

find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------------------
# Core library
# ---------------------------------------------------------------------------------------
file(GLOB_RECURSE AETHER_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(aether ${AETHER_SOURCES})
target_include_directories(aether PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(aether PUBLIC Eigen3::Eigen Threads::Threads PRIVATE ${AETHER_YAML_TARGET})

# Shared warning configuration, applied to every target built from this project's own
# sources (the library, the applications and the tests) but never to fetched dependencies.
function(aether_set_warnings target)
  if(AETHER_ENABLE_WARNINGS)
    if(MSVC)
      target_compile_options(${target} PRIVATE /W4)
    else()
      target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow
                                               -Wnon-virtual-dtor -Wcast-align -Wunused
                                               -Woverloaded-virtual -Wdouble-promotion)
    endif()
  endif()
  if(AETHER_ENABLE_WERROR AND NOT MSVC)
    target_compile_options(${target} PRIVATE -Werror)
  endif()
endfunction()

aether_set_warnings(aether)

# ---------------------------------------------------------------------------------------
# Applications
# ---------------------------------------------------------------------------------------
set(AETHER_APPS aether_sim aether_trim aether_mc aether_integrators)
foreach(app IN LISTS AETHER_APPS)
  add_executable(${app} apps/${app}.cpp)
  target_link_libraries(${app} PRIVATE aether)
  aether_set_warnings(${app})
  set_target_properties(${app} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endforeach()

# ---------------------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------------------
if(AETHER_BUILD_TESTS)
  enable_testing()
  find_package(Catch2 3 QUIET)
  if(NOT Catch2_FOUND)
    message(STATUS "Catch2 v3 not found on the system - fetching it")
    FetchContent_Declare(Catch2
      GIT_REPOSITORY https://github.com/catchorg/Catch2.git
      GIT_TAG v3.5.2
      GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(Catch2)
    list(APPEND CMAKE_MODULE_PATH ${Catch2_SOURCE_DIR}/extras)
  else()
    if(EXISTS "${Catch2_DIR}/../../../share/Catch2/extras")
      list(APPEND CMAKE_MODULE_PATH "${Catch2_DIR}/../../../share/Catch2/extras")
    endif()
  endif()

  file(GLOB AETHER_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
  add_executable(aether_tests ${AETHER_TEST_SOURCES})
  target_link_libraries(aether_tests PRIVATE aether Catch2::Catch2WithMain)
  aether_set_warnings(aether_tests)
  target_compile_definitions(aether_tests PRIVATE
    AETHER_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/configs")
  set_target_properties(aether_tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

  include(CTest)
  find_package(Catch2 3 QUIET)
  include(Catch OPTIONAL RESULT_VARIABLE AETHER_CATCH_MODULE)
  if(AETHER_CATCH_MODULE)
    catch_discover_tests(aether_tests)
  else()
    add_test(NAME aether_tests COMMAND aether_tests)
  endif()
endif()

message(STATUS "aether6 ${PROJECT_VERSION}: build type = ${CMAKE_BUILD_TYPE}, tests = ${AETHER_BUILD_TESTS}")
