set(FILTER_NAME objectFilter)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})

# add the include directory of the filter
include_directories(../include)
# declare our filter
adtf_add_filter(objectFilter
    stdafx.h
    stdafx.cpp
    
    cvBlob/cvblob.h
    cvBlob/cvblob.cpp
    cvBlob/cvaux.cpp
    cvBlob/cvcolor.cpp
    cvBlob/cvcontour.cpp
    cvBlob/cvlabel.cpp
    cvBlob/cvtrack.cpp

    object_filter.h
    object_filter.cpp
    ../util/object.h
    ../util/object.cpp
    ../util/vector2.h
    ../util/vector2.cpp
    ../util/point_transformer.h
    ../util/point_transformer.cpp
)

target_link_libraries(objectFilter ${OpenCV_LIBS})
adtf_install_plugin(objectFilter ${AADC_INSTALL_DIR})
