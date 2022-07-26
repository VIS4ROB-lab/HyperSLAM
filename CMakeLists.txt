# Check CMake version.
cmake_minimum_required(VERSION 3.16)
enable_language(CXX)

# Includes.
include(FetchContent)

# Define projects.
project(HyperSystem VERSION 22.07.04 DESCRIPTION "" HOMEPAGE_URL https://github.com/VIS4ROB-lab/HyperSLAM LANGUAGES CXX)

# Define libraries and aliases.
add_library(${PROJECT_NAME} INTERFACE)
add_library(Hyper::System ALIAS ${PROJECT_NAME})

# Set properties.
set_target_properties(${PROJECT_NAME} PROPERTIES INTERFACE_CXX_STANDARD 20 INTERFACE_CXX_STANDARD_REQUIRED YES)
target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Wextra -Wnon-virtual-dtor -pedantic)
target_compile_options(${PROJECT_NAME} INTERFACE -std=c++2a -march=native -O3 -fconcepts)

# Fetch (external) content.
FetchContent_Declare(HyperSensors GIT_REPOSITORY https://github.com/VIS4ROB-lab/HyperSensors.git GIT_TAG ef63793c10af46f393d5d7351ddad64fd99495d0)
FetchContent_MakeAvailable(HyperSensors)

# Find Eigen target if not present.
if (NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 REQUIRED NO_MODULE)
endif ()

# Find GTest target if not present.
if (NOT TARGET GTest::GTest)
    find_package(GTest REQUIRED)
endif ()

# Find glog target if not present.
if (NOT TARGET glog::glog)
    find_package(glog REQUIRED)
endif ()

# Find roscpp.
if (NOT roscpp_FOUND)
    find_package(roscpp REQUIRED)
endif ()

# Find std_msgs.
if (NOT std_msgs_FOUND)
    find_package(std_msgs REQUIRED)
endif ()

# Find sensor_msgs.
if (NOT sensor_msgs_FOUND)
    find_package(sensor_msgs REQUIRED)
endif ()

# Find cv_bridge.
if (NOT cv_bridge_FOUND)
    find_package(cv_bridge REQUIRED)
endif ()

# Find OpenCV.
if (NOT OpenCV_FOUND)
    find_package(OpenCV REQUIRED)
endif ()

# Find yaml-cpp.
if (NOT yaml-cpp_FOUND)
    find_package(yaml-cpp REQUIRED)
endif ()

# Add includes.
target_include_directories(
        ${PROJECT_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        ${roscpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${yaml-cpp_INCLUDE_DIRS}
)

# Add sources.
file(GLOB_RECURSE PROJECT_SOURCE_FILES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/include/*.hpp ${PROJECT_SOURCE_DIR}/internal/*.hpp ${PROJECT_SOURCE_DIR}/internal/*.cpp)
target_sources(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_FILES})

# Link libraries.
target_link_libraries(
        ${PROJECT_NAME}
        INTERFACE
        Hyper::Sensors
        ${roscpp_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${cv_bridge_LIBRARIES}
        ${OpenCV_LIBRARIES}
        yaml-cpp
)

# Compile HyperSLAM.
add_subdirectory(apps/hyperslam)

# Compile tests if requested.
option(HYPER_ENABLE_TESTING "Build tests." ON)
if (HYPER_ENABLE_TESTING)
    add_subdirectory(apps/tests)
endif ()

# Compile benchmarks if requested.
option(HYPER_ENABLE_BENCHMARKS "Build benchmarks." ON)
if (HYPER_ENABLE_BENCHMARKS)
    add_subdirectory(apps/benchmarks)
endif ()
