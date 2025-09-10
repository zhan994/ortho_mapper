cmake_minimum_required(VERSION 3.0.2)
project(ortho_mapper)

# for Debug
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TIFF REQUIRED)
find_package(jsoncpp REQUIRED)
set(JSONCPP_LIB jsoncpp_lib) # no exported jsoncpp_lib

aux_source_directory(src/ SOURCES)
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} 
  ${JSONCPP_LIB}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${TIFF_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} 
  PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)