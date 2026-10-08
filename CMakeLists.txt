cmake_minimum_required(VERSION 3.16)
project(tirocinio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZEROMQ REQUIRED libzmq)

include_directories(${ZEROMQ_INCLUDE_DIRS} . coppelia_api)

add_executable(test_connection 
    main.cpp 
    coppelia_api/RemoteAPIClient.cpp 
    coppelia_api/RemoteAPIObjects.cpp
)

target_link_libraries(test_connection PRIVATE ${ZEROMQ_LIBRARIES})