cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 11)
project(JrIntegration)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
message(WARNING ${CMAKE_MODULE_PATH})
find_package(Phobos REQUIRED)

include_directories(
	include
    ${Phobos_INCLUDE_DIRS}
    ${Opencv_INCLUDE_DIRS}
)

add_executable(
	test_JR src/test_JR.cpp src/matcherJrsgm.cpp src/iniReader.cpp
)

target_link_libraries(
	test_JR
	${Phobos_LIBRARIES}
    ${OpenCV_LIBRARIES}
)