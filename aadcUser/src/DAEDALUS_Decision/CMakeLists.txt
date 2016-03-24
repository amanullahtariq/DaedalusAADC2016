# add the include directory of the filter
set(FILTER_NAME aadc_daedalus_decision)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})

include_directories(../include)
include_directories(../DAEDALUS_util)
# declare our filter



adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp
    daedalusDecision.h
    daedalusDecision.cpp
     IPM.h
    IPM.cpp
    ParkingType.h
    ParkingType.cpp
    ../DAEDALUS_util/DAEDALUS_util.h
    ../DAEDALUS_util/DAEDALUS_util.cpp
    ../utile/object/object.h
    ../utile/object/object.cpp
)

target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

adtf_set_folder(${FILTER_NAME} opencv) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
