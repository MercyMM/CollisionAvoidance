#	CMake buildchain for COLA
#-------------------------------

cmake_minimum_required(VERSION 2.6)
project(COLA)

cmake_policy(SET CMP0003 NEW)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_definitions( -fPIC -msse2 -mssse3 -march=nocona)

find_package(OpenCV REQUIRED)

#agast
add_subdirectory (${PROJECT_SOURCE_DIR}/thirdparty/agast)
include_directories (${PROJECT_SOURCE_DIR}/thirdparty/agast/include)

#brisk
add_subdirectory (${PROJECT_SOURCE_DIR}/thirdparty/brisk)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/brisk/include)

#COLA
include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(cola_demo src/main.cpp src/KeyFeature.cpp src/OF.cpp src/openCVhelpers.cpp src/ui.cpp src/GraphUtils.cpp)
target_link_libraries(cola_demo brisk agast ${OpenCV_LIBS})
