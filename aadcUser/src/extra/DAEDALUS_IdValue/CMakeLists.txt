# add the include directory of the filter
set(FILTER_NAME daedalus_idvaluesender)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})



adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp
    daedalus_idvaluesender.h
    daedalus_idvaluesender.cpp
       )

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
