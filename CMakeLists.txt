project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall -Werror -Wextra -pedantic")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
# set(CMAKE_VERBOSE_MAKEFILE ON)

set(sources src/main.cpp src/traffic.cpp src/cost.cpp src/map.cpp src/path_planning.cpp src/helpers.cpp)
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
