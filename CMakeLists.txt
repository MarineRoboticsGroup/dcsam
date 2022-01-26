cmake_minimum_required(VERSION 3.10)
project(dcsam)

# Set some compilation options.
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wpedantic -Wextra)

# Add option to enable testing
option(ENABLE_TESTS "Enable tests" OFF)

# External package dependencies.
find_package(GTSAM 4.1 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

# Testing on MacOS
# find_package(TBB 4.4 COMPONENTS tbb tbbmalloc)

# add_definitions(-march=native)
# add_definitions(-std=c++1z)

set(DCSAM_HDRS
  include/dcsam/DCSAM_types.h
  include/dcsam/DCSAM_utils.h
  include/dcsam/DCSAM.h
  include/dcsam/DCFactor.h
  include/dcsam/DCContinuousFactor.h
  include/dcsam/DCDiscreteFactor.h
  include/dcsam/DCMixtureFactor.h
  include/dcsam/DiscretePriorFactor.h
  include/dcsam/SmartDiscretePriorFactor.h
  include/dcsam/HybridFactorGraph.h
  include/dcsam/SemanticBearingRangeFactor.h
  include/dcsam/SmartSemanticBearingRangeFactor.h
  include/dcsam/DCMaxMixtureFactor.h
  include/dcsam/DCEMFactor.h
  include/dcsam/DiscreteMarginalsOrdered.h
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
if (${ENABLE_TESTS})
  message(STATUS "Testing enabled. Building tests.")
  enable_testing()
  add_subdirectory(tests)
endif()
