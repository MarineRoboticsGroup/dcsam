cmake_minimum_required(VERSION 3.10)
project(dcsam)

# Set some compilation options.
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wpedantic -Wextra)

# External package dependencies.
find_package(GTSAM 4.1 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

# add_definitions(-march=native)

set(DCSAM_HDRS
  include/dcsam/DCSAM.h
  include/dcsam/DCSAM_types.h
  include/dcsam/DCFactor.h
  include/dcsam/DCContinuousFactor.h
  include/dcsam/DCDiscreteFactor.h
  include/dcsam/DCMixtureFactor.h
  include/dcsam/DiscretePriorFactor.h
  include/dcsam/SmartDiscretePriorFactor.h
  include/dcsam/HybridFactorGraph.h
  )

set(DCSAM_SRCS
  src/DCSAM.cpp
  src/HybridFactorGraph.cpp
  )

set(DCSAM_LIBS
  gtsam
  Eigen3::Eigen
  )

add_library(dcsam STATIC ${DCSAM_SRCS} ${DCSAM_HDRS})
target_include_directories(dcsam PUBLIC include)
target_link_libraries(dcsam PUBLIC ${DCSAM_LIBS})

# Make library accessible to other cmake projects
export(PACKAGE dcsam)
export(TARGETS dcsam FILE dcsamConfig.cmake)

# Include unit tests directory to the project.
enable_testing()
add_subdirectory(tests)
