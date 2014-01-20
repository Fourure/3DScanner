cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(openni_grabber)

find_package(OpenCV	REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS})

add_executable(openni_grabber
  src/openni_grabber.cpp
)

target_link_libraries(openni_grabber
  ${OpenCV_LIBS}
)

