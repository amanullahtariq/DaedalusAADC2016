set(FILTER_NAME daedalus_DriverModule)
include_directories(${OpenCV_INCLUDE_DIR})
include_directories(../DAEDALUS_util)
   
adtf_add_filter(${FILTER_NAME}
            stdafx.cpp
            stdafx.h
            juryEnums.h
            daedalusDriverModule.cpp            
            daedalusDriverModule.h
            Maneuverlist.h
		 	   ../DAEDALUS_util/DAEDALUS_util.h
		 	   ../DAEDALUS_util/DAEDALUS_util.cpp
            ${HEADER_MOC}
)

# Link against the libraries
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/stateController) 

adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
    




