#ifndef _OBJECT_FILTER_H_
#define _OBJECT_FILTER_H_

#define OID_ADTF_OBJECT_FILTER  "dev.fruit.object_filter"

#include "../util/object.h"

class ObjectFilter : public adtf::cFilter {
  ADTF_FILTER(OID_ADTF_OBJECT_FILTER, "DaedALUs Object Filter", adtf::OBJCAT_DataFilter);
 
 public:
 	ObjectFilter(const tChar* __info);
 	virtual ~ObjectFilter();
 	
 	tResult Init(tInitStage stage, __exception);
  tResult Shutdown(tInitStage stage, __exception);
	tResult OnPinEvent(IPin* source, tInt event_code, tInt param1, tInt param2,
    IMediaSample* media_sample);
 
 protected:
  // Depth image inpu pin
 	cVideoPin video_input_pin_;
 	tBitmapFormat input_format_;
 	
 	// Object image output pin
 	cVideoPin video_output_pin_;
 	tBitmapFormat output_format_;
 	
 	// Object data output pin
 	cOutputPin object_output_pin_;
 	cOutputPin mapped_object_output_pin_;
 	
 	cObjectPtr<IMediaTypeDescription> object_data_description_;
 	
 private:
  tResult processImage(IMediaSample* sample);
  tResult transmitObjects(std::vector<Object> const & objects, cOutputPin & output_pin);
};

#endif  // _OBJECT_FILTER_H_
