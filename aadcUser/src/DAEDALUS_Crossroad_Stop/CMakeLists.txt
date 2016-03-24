# add the include directory of the filter
include_directories(../include)
include_directories(${OpenCV_INCLUDE_DIR})
include_directories(../extra/IPM)
include_directories(../extra/Map)
include_directories(../extra/Landmarks)
include_directories(../DAEDALUS_util)

# declare our filter
set(FILTER_NAME DAEDALUS_Crossroad_Stop_Filter)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    ../extra/IPM/IPM.h
	../extra/IPM/IPM.cpp
	../extra/Landmarks/Landmark.h
	../extra/Landmarks/Landmark.cpp
	../extra/Map/Map.h
	../extra/Map/Map.cpp
	../DAEDALUS_util/DAEDALUS_util.h
	../DAEDALUS_util/DAEDALUS_util.cpp

    DAEDALUS_Crossroad_Stop.h
    DAEDALUS_Crossroad_Stop.cpp
)

# Link agains the libraries
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
