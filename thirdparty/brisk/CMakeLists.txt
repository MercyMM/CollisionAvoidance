cmake_minimum_required(VERSION 2.4.6)

file(GLOB BRISK_SOURCE_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

file(GLOB BRISK_HEADER_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/include/brisk/*.h")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include/brisk)

#build the library dynamic and static version
add_library(brisk ${BRISK_SOURCE_FILES} ${BRISK_HEADER_FILES})
