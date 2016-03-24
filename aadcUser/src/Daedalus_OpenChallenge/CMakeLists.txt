set(FILTER_NAME OpenChallenge)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

include_directories(${OpenCV_INCLUDE_DIR})
include_directories(../DAEDALUS_util)

# add the include directory of the filter
include_directories(../include)
# declare our filter
adtf_add_filter(OpenChallenge
    stdafx.h
    stdafx.cpp
    OpenChallenge.h
    OpenChallenge.cpp
    ../DAEDALUS_util/DAEDALUS_util.h
	../DAEDALUS_util/DAEDALUS_util.cpp

)

target_link_libraries(OpenChallenge ${OpenCV_LIBS})
adtf_install_plugin(OpenChallenge ${AADC_INSTALL_DIR})
