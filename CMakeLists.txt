cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(openni_grabber)

find_package(OpenCV	REQUIRED)
find_package(PCL		REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(
  ${PCL_DEFINITIONS}
)
set(CMAKE_INSTALL_RPATH ${PCL_LIBRARY_DIRS})

add_executable( openni_grabber src/openni_grabber.cpp )
target_link_libraries(openni_grabber  ${PCL_LIBRARIES} ${OpenCV_LIBS} )

