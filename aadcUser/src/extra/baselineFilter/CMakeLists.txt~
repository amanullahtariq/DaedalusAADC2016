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
