# Declare project.
project(HyperSLAM VERSION 22.08.17 DESCRIPTION "" HOMEPAGE_URL https://github.com/VIS4ROB-lab/HyperSLAM LANGUAGES CXX)

# Add executable.
add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/main.cpp)

# Link libraries.
target_link_libraries(${PROJECT_NAME} PRIVATE Hyper::System)
