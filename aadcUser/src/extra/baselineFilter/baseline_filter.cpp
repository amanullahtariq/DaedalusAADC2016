#include "stdafx.h"
#include "baseline_filter.h"
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>

using namespace cv;

ADTF_FILTER_PLUGIN("FRUIT Baseline Filter", OID_ADTF_BASELINE_FILTER, BaselineFilter);

// -------------------------------------------------------------------------------------------------
BaselineFilter::BaselineFilter(const tChar* __info) {
// -------------------------------------------------------------------------------------------------
  counter_ = 0;
  
  cMemoryBlock::MemSet(&input_format_, 0, sizeof(input_format_));
  
  SetPropertyStr("path", "/home/aadc/Desktop/images/base/27");
}

// -------------------------------------------------------------------------------------------------
BaselineFilter::~BaselineFilter() {
// -------------------------------------------------------------------------------------------------

}

// -------------------------------------------------------------------------------------------------
tResult BaselineFilter::Init(tInitStage stage, __exception) {
// -------------------------------------------------------------------------------------------------
  RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr));
  
	if (stage == StageFirst) {
		// Create and register the video input pin
		RETURN_IF_FAILED(video_input_pin_.Create("Video_Input", IPin::PD_Input,
		  static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&video_input_pin_));
	}
	
	else if (stage == StageGraphReady) {
	  std::stringstream ss;
	  ss << GetPropertyStr("path");
	  std::string p = ss.str();
	  if (p.at(p.length() - 1) != '/') SetPropertyStr("path", p.append("/").c_str());
	
	  cObjectPtr<IMediaType> type;
	  RETURN_IF_FAILED(video_input_pin_.GetMediaType(&type));
	  
	  cObjectPtr<IMediaTypeVideo> type_video;
	  RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**) &type_video));
	  
	  const tBitmapFormat* format = type_video->GetFormat();
    cMemoryBlock::MemCopy(&input_format_, format, sizeof(tBitmapFormat));
	}
	
	RETURN_NOERROR;
}

// -------------------------------------------------------------------------------------------------
tResult BaselineFilter::Shutdown(tInitStage stage, __exception) {
// -------------------------------------------------------------------------------------------------
	return cFilter::Shutdown(stage,__exception_ptr);
}

// -------------------------------------------------------------------------------------------------
tResult BaselineFilter::OnPinEvent(IPin* source, tInt event_code, tInt param1, tInt param2,
    IMediaSample* media_sample) {
// -------------------------------------------------------------------------------------------------
	RETURN_IF_POINTER_NULL(source);
	RETURN_IF_POINTER_NULL(media_sample);
	
	if (event_code == IPinEventSink::PE_MediaSampleReceived) {
		if (source == &video_input_pin_) {
      processImage(media_sample);
		}
	}
		
	RETURN_NOERROR;
}

// -------------------------------------------------------------------------------------------------
tResult BaselineFilter::processImage(IMediaSample* sample) {
// -------------------------------------------------------------------------------------------------
  // Check if the sample is valid
  RETURN_IF_POINTER_NULL(sample);
  
  // Initialize the data buffers
  const tVoid* source_buffer;
  
  if (IS_OK(sample->Lock(&source_buffer))) {
    // Create the source image matrix    
    Mat source_image(input_format_.nHeight, input_format_.nWidth, CV_8UC2, (uchar*)source_buffer);
    
    // Retrieve the actual depth image
    Mat source_channels[2];
    split(source_image, source_channels);
    Mat depth_image = source_channels[1];
    
    // Merge white and black noise
    for (int i = 0; i < depth_image.rows; i++) {
      for (int j = 0; j < depth_image.cols; j++) {
        if (depth_image.at<uchar>(i,j) == 255) depth_image.at<uchar>(i,j) = 0;
      }
    }
    
    std::stringstream ss;
    ss << GetPropertyStr("path") << counter_ << ".png";
    counter_++;
    imwrite(ss.str().c_str(), depth_image);
    
    LOG_INFO(cString::Format("Saved image #%d", counter_));
    
    sample->Unlock(source_buffer);
  }
  
  RETURN_NOERROR;
}
