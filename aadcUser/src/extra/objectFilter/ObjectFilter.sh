#!/bin/sh

export DISPLAY=:0
export ADTF_DIR=/opt/adtf/2.13.1
#adjust to path of adtf config file
export ADTF_PROJECT_PATH=/home/aadc/object_project/object_filter/object_filter.prj
export ADTF_CONFIG_PATH=/home/aadc/object_project/object_filter/config/system.xml


if [ -d $ADTF_DIR ]; then
    echo "ADTF dir found."
else
    echo "ADTF dir not found in $ADTF_DIR. Check the path to ADTF."
    exit 1
fi


$ADTF_DIR/bin/adtf_devenv -project=$ADTF_PROJECT_PATH -run

#the qt filters and the video displays have to be deactivated before
#$ADTF_DIR/bin/adtf_runtime -project=$ADTF_PROJECT_PATH -minimized -run


