cmake_minimum_required (VERSION 3.5.0)
project (haption)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                ${PROJECT_SOURCE_DIR}/bin)

link_directories(${PROJECT_SOURCE_DIR}/linux/lib/x64)
link_directories(${PROJECT_SOURCE_DIR}/linux/bin/x64)
link_directories(${PROJECT_SOURCE_DIR}/linux/lib/x86)
link_directories(${PROJECT_SOURCE_DIR}/linux/bin/x86)
add_executable (haption haption.cpp)
target_link_libraries(
        haption
        virtuose
        dl
)