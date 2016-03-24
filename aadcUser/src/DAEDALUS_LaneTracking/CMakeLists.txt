set(FILTER_NAME daedalus_laneTracking)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})


adtf_add_filter(${FILTER_NAME}
    stdafx.h

    DAEDALUS_LaneTracking.h
    DAEDALUS_LaneTracking.cpp
)


target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

adtf_set_folder(${FILTER_NAME} daedalustraffic) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
