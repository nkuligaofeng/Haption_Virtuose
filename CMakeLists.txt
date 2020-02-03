cmake_minimum_required (VERSION 3.5.0)
project (haption)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                ${PROJECT_SOURCE_DIR}/bin)

link_directories(${PROJECT_SOURCE_DIR}/VirtuoseLib/lib/x64)
link_directories(${PROJECT_SOURCE_DIR}/VirtuoseLib/bin/x64)
add_executable (haption haption.cpp)
target_link_libraries(
        haption
        virtuose
        dl
)