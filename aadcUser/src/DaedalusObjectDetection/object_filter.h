#ifndef _OBJECT_FILTER_H_
#define _OBJECT_FILTER_H_

#define OID_ADTF_OBJECT_DETECTION  "daedalus.object.detection"

#include "../extra/util/object.h"
// #include <pthread.h>

class ObjectFilter : public adtf::cFilter
{
  ADTF_FILTER(OID_ADTF_OBJECT_DETECTION, "Daedalus Object Detection", adtf::OBJCAT_DataFilter);
 
 public:
    ObjectFilter(const tChar* __info);
    virtual ~ObjectFilter();
    
    tResult Init(tInitStage stage, __exception);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult Shutdown(tInitStage eStage, __exception = NULL);   
    tResult OnPinEvent(IPin* source, tInt event_code, tInt param1, tInt param2,
    IMediaSample* media_sample);
 
 protected:
  // Depth image inpu pin
    cVideoPin video_input_pin_;
    tBitmapFormat input_format_;
    
    // Object image output pin
    // cVideoPin video_output_pin_;
    // tBitmapFormat output_format_;
    
    // Object data output pin
    cOutputPin object_output_pin_;
    // cOutputPin mapped_object_output_pin_;
    
    cObjectPtr<IMediaTypeDescription> object_data_description_;

    // pthread_mutex_t m_input_mux;
    
 private:
  tResult ProcessImage(IMediaSample* sample);
  tResult TransmitObjects(std::vector<Object> const & objects);
  /*! creates all the input Pins*/
  tResult CreateInputPins(__exception = NULL);
  /*! creates all the output Pins*/
  tResult CreateOutputPins(__exception = NULL);
  /*! creates all the filter propertys*/
  tVoid SetAllProperties();
  tVoid GetAllProperties();
  tVoid Initialize();

private:
    int horizon_base;
    int horizon_source;
    int min_blob_size;
    int max_blob_size;
    int offset_hor;
    int white_threshold;
    int base_threshold;
    tFloat32 scale_height;
    tFloat32 scale_width;
    bool logModeEnabled;
    cCriticalSection  TransmitObjectsCricSection;

};

#endif  // _OBJECT_FILTER_H_
