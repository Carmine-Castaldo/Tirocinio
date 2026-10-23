cmake_minimum_required(VERSION 3.16)
project(tirocinio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-g -O0)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZEROMQ REQUIRED libzmq)
find_package(behaviortree_cpp REQUIRED)

include_directories(
    ${ZEROMQ_INCLUDE_DIRS}
    .
    headers
    coppelia_api
)

file(GLOB SOURCES_CPP
    "sources/*.cpp"
    "coppelia_api/*.cpp"
)

list(REMOVE_ITEM SOURCES_CPP 
    "${CMAKE_CURRENT_SOURCE_DIR}/coppelia_api/sim-deprecated.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/coppelia_api/sim-special.cpp"
)

add_executable(main 
    main.cpp 
    ${SOURCES_CPP}
)
target_link_libraries(main PRIVATE 
    zmq
    BT::behaviortree_cpp
)