# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME user_filterDrive)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    Filter_Drive.h
    Filter_Drive.cpp
)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
