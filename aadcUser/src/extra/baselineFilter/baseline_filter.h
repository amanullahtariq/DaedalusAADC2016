#ifndef _BASELINE_FILTER_H_
#define _BASELINE_FILTER_H_

#define OID_ADTF_BASELINE_FILTER  "fruit.dev.baseline_filter"

#include <stdio.h>

class BaselineFilter : public adtf::cFilter {
  ADTF_FILTER(OID_ADTF_BASELINE_FILTER, "FRUIT Baseline Filter", adtf::OBJCAT_DataFilter);
 
 public:
 	BaselineFilter(const tChar* __info);
 	virtual ~BaselineFilter();
 	
 	tResult Init(tInitStage stage, __exception);
  tResult Shutdown(tInitStage stage, __exception);
	tResult OnPinEvent(IPin* source, tInt event_code, tInt param1, tInt param2,
    IMediaSample* media_sample);
 
 protected:
  // Depth image inpu pin
 	cVideoPin video_input_pin_;
 	tBitmapFormat input_format_;
 	
 private:
  tResult processImage(IMediaSample* sample);
  size_t counter_;
};

#endif  // _BASELINE_FILTER_H_
