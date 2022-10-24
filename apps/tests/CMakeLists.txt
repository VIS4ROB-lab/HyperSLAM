# Declare project.
project(HyperSystemTests VERSION 22.09.08 DESCRIPTION "" HOMEPAGE_URL https://github.com/VIS4ROB-lab/HyperVariables LANGUAGES CXX)

# Collect files and add executable.
get_filename_component(APPS_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)
get_filename_component(PARENT_DIR ${APPS_DIR} DIRECTORY)
file(GLOB_RECURSE PROJECT_SOURCE_FILES CONFIGURE_DEPENDS ${PARENT_DIR}/tests/*.hpp ${PARENT_DIR}/tests/*.cpp)
add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/main.cpp ${PROJECT_SOURCE_FILES})

# Add includes and link libraries.
target_include_directories(${PROJECT_NAME} PRIVATE ${PARENT_DIR}/tests/include)
target_link_libraries(${PROJECT_NAME} Hyper::System)
