option(ENABLE_PLOTTING "Enable plotting (requires matplotlibcpp)" OFF)

if (ENABLE_PLOTTING)
find_package(matplotlibcpp REQUIRED)
endif()

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

if (ENABLE_PLOTTING)
  add_definitions(-DENABLE_PLOTTING=1)
endif()

# Include tests here...
add_executable(testDCSAM testDCSAM.cpp)

# If plotting is enabled, we need to link to matplotlibcpp
if (ENABLE_PLOTTING)
  target_link_libraries(testDCSAM dcsam gtest gtest_main matplotlibcpp)
else()
  target_link_libraries(testDCSAM dcsam /usr/local/lib/libgtest.a /usr/local/lib/libgtest_main.a gtsam)
endif()

add_test(TestDCSAM testDCSAM)
