cmake_minimum_required(VERSION 3.0.2)
project(ortho_mapper)

# for Debug
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE "Release")
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  "/usr/include/eigen3"
  "/usr/include/jsoncpp"
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

aux_source_directory(src/ SOURCES)
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} 
  jsoncpp
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)