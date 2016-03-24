set(FILTER_NAME ParkingspotDetection)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})

include_directories(../include)
include_directories(../DAEDALUS_util)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    parkingspot_detection.h
    parkingspot_detection.cpp
    IPM.h
    IPM.cpp
    ParkingType.h
    ParkingType.cpp
    ../DAEDALUS_util/DAEDALUS_util.h
    ../DAEDALUS_util/DAEDALUS_util.cpp
)


target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

adtf_set_folder(${FILTER_NAME} opencv) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
