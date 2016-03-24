set(FILTER_NAME baselineFilter)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})

# add the include directory of the filter
include_directories(../include)
# declare our filter
adtf_add_filter(baselineFilter
    stdafx.h
    stdafx.cpp
    
    baseline_filter.h
    baseline_filter.cpp
)

target_link_libraries(baselineFilter ${OpenCV_LIBS})
adtf_install_plugin(baselineFilter ${AADC_INSTALL_DIR})
