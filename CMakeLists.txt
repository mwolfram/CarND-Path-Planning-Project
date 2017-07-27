project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources
    src/planner.cpp
    src/configuration.cpp
    )

set(main-sources ${sources} src/main.cpp)
#set(debug-sources ${sources} debug/debug.cpp)
set(test-sources ${sources} test/test_main.cpp
    test/test_factorial.cpp
    test/test_configuration.cpp
    test/test_toolkit.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${main-sources})
target_link_libraries(path_planning z ssl uv uWS)

#add_executable(path_planning_debug ${debug-sources})
#target_link_libraries(path_planning_debug z ssl uv uWS)

add_executable(path_planning_test ${test-sources})
target_link_libraries(path_planning_test z ssl uv uWS)
