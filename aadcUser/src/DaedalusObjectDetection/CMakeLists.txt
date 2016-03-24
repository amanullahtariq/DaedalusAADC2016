set(FILTER_NAME ObjectDeteciton)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})

# add the include directory of the filter
include_directories(../include)
# declare our filter
adtf_add_filter(ObjectDeteciton
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
    ../extra/util/object.h
    ../extra/util/object.cpp
    ../extra/util/vector2.h
    ../extra/util/vector2.cpp
    ../extra/util/point_transformer.h
    ../extra/util/point_transformer.cpp
)

target_link_libraries(ObjectDeteciton ${OpenCV_LIBS})
adtf_install_plugin(ObjectDeteciton ${AADC_INSTALL_DIR})
