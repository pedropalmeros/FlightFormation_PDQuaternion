PROJECT(MultiAgent_simulator)
cmake_minimum_required(VERSION 2.8)

SET(FLAIR_USE_SIMULATOR TRUE)
SET(FLAIR_USE_SIMULATOR_GL TRUE)

include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)

SET(SRC_FILES
	src/main.cpp
)

FLAIR_DEMO(${PROJECT_NAME} "${SRC_FILES}"
           DEST_DIR SimpleFleet
           COPY_RESOURCES
)
