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
  FetchContent_Declare(matplotlibcpp
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG ef0383f1315d32e0156335e10b82e90b334f6d9f
  )

  FetchContent_GetProperties(matplotlibcpp)
  if(NOT matplotlibcpp_POPULATED)
    FetchContent_Populate(matplotlibcpp)

    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
    find_package(Python3 COMPONENTS NumPy)

    add_library(matplotlibcpp INTERFACE)
    target_include_directories(matplotlibcpp INTERFACE ${matplotlibcpp_SOURCE_DIR})
    target_compile_features(matplotlibcpp INTERFACE cxx_std_11)
    target_link_libraries(matplotlibcpp INTERFACE Python3::Python Python3::Module)
    if(Python3_NumPy_FOUND)
      target_link_libraries(matplotlibcpp INTERFACE Python3::NumPy)
    else()
      target_compile_definitions(matplotlibcpp INTERFACE WITHOUT_NUMPY)
    endif()
  endif()

  target_compile_definitions(testDCSAM PRIVATE ENABLE_PLOTTING=1)
  target_link_libraries(testDCSAM matplotlibcpp)
endif()

add_test(TestDCSAM testDCSAM)
