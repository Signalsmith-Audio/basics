cmake_minimum_required(VERSION 3.24)

add_library(signalsmith-basics INTERFACE)
target_include_directories(signalsmith-basics INTERFACE
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

include(FetchContent)
FetchContent_Declare(
	signalsmith-linear
	GIT_REPOSITORY https://github.com/Signalsmith-Audio/linear.git
	GIT_TAG 0.3.0
	GIT_SHALLOW ON
)
FetchContent_Declare(
	signalsmith-dsp
	GIT_REPOSITORY https://github.com/Signalsmith-Audio/dsp.git
	GIT_TAG v1.7.0
	GIT_SHALLOW ON
)
FetchContent_Declare(
	signalsmith-hilbert
	GIT_REPOSITORY https://github.com/Signalsmith-Audio/hilbert-iir.git
	GIT_TAG 1.0.0
	GIT_SHALLOW ON
)
FetchContent_MakeAvailable(signalsmith-linear signalsmith-dsp signalsmith-hilbert)

target_link_libraries(signalsmith-basics INTERFACE
	signalsmith-linear signalsmith-dsp signalsmith-hilbert
)
