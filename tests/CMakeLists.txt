option(ENABLE_PLOTTING "Enable plotting (requires matplotlibcpp)" OFF)

if (ENABLE_PLOTTING)
  message(STATUS "Plotting enabled. Building tests with plotting.")
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

if (APPLE)
  set(GTEST_LIBS
    /usr/local/lib/libgtest.a
    /usr/local/lib/libgtest_main.a
  )
else()
  set(GTEST_LIBS
    gtest
    gtest_main
)
endif()



# If plotting is enabled, we need to link to matplotlibcpp
if (ENABLE_PLOTTING)
  target_link_libraries(testDCSAM dcsam matplotlibcpp gtsam ${GTEST_LIBS})
else()
  target_link_libraries(testDCSAM dcsam gtsam ${GTEST_LIBS})
endif()

add_test(TestDCSAM testDCSAM)
