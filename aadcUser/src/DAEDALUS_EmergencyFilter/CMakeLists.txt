set(FILTER_NAME EmergencyFilter)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})

# add the include directory of the filter
include_directories(../include)
# declare our filter
adtf_add_filter(EmergencyFilter
    stdafx.h
    stdafx.cpp
    
    EmergencyFilter.h
    EmergencyFilter.cpp

)

target_link_libraries(EmergencyFilter ${OpenCV_LIBS})
adtf_install_plugin(EmergencyFilter ${AADC_INSTALL_DIR})
