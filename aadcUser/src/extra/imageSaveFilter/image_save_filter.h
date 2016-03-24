#ifndef _IMAGE_SAVE_FILTER_H_
#define _IMAGE_SAVE_FILTER_H_

#define OID_ADTF_IMAGE_SAVE_FILTER  "fruit.dev.image_save_filter"

class ImageSaveFilter : public adtf::cFilter {
  ADTF_FILTER(OID_ADTF_IMAGE_SAVE_FILTER, "Image Save Filter", adtf::OBJCAT_DataFilter);
 
 public:
 	ImageSaveFilter(const tChar* __info);
 	virtual ~ImageSaveFilter();
 	
 	tResult Init(tInitStage stage, __exception);
  tResult Shutdown(tInitStage stage, __exception);
	tResult OnPinEvent(IPin* source, tInt event_code, tInt param1, tInt param2,
    IMediaSample* media_sample);
 
 protected:
  // Input pin
 	cVideoPin video_input_pin_;
 	tBitmapFormat input_format_;
 	
 	// Output pin
 	cVideoPin video_output_pin_;
 	tBitmapFormat output_format_;
 	
 private:
  tResult setHeightAndWidth();
  tResult processImage(IMediaSample* sample);
  
 	tInt height_, width_;
};

#endif  // _IMAGE_SAVE_FILTER_H_
