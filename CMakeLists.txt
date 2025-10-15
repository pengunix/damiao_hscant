cmake_minimum_required(VERSION 3.10.0)
project(dm_motor VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(dm_motor 
    ${PROJECT_SOURCE_DIR}/src/test_damiao.cpp
    ${PROJECT_SOURCE_DIR}/src/damiao.cpp    
)
target_link_libraries(dm_motor pthread)
