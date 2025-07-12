cmake_minimum_required(VERSION 3.24)

add_library(signalsmith-basics INTERFACE)
target_include_directories(signalsmith-basics INTERFACE
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/modules
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/modules/linear)
target_link_libraries(signalsmith-basics INTERFACE
	signalsmith-linear
)
