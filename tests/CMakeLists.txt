option(DCSAM_ENABLE_PLOTTING "Enable plotting (requires matplotlib_cpp)" OFF)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if(CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

if(APPLE)
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

add_executable(testDCSAM testDCSAM.cpp)
target_link_libraries(testDCSAM dcsam gtsam ${GTEST_LIBS})

# If plotting is enabled, we need to link to matplotlib_cpp
if(DCSAM_ENABLE_PLOTTING)
  message(STATUS "Plotting enabled. Building tests with plotting.")
  include(FetchContent)
  FetchContent_Declare(matplotlib_cpp
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG ef0383f1315d32e0156335e10b82e90b334f6d9f
  )
  FetchContent_MakeAvailable(matplotlib_cpp)

  target_compile_definitions(testDCSAM PRIVATE ENABLE_PLOTTING=1)
  target_link_libraries(testDCSAM matplotlib_cpp)
  # workaround until: https://github.com/lava/matplotlib-cpp/pull/335
  target_include_directories(testDCSAM PRIVATE ${matplotlib_cpp_SOURCE_DIR})
endif()

add_test(TestDCSAM testDCSAM)
