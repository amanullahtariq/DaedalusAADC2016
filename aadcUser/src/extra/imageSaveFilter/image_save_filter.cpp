#include "stdafx.h"
#include "image_save_filter.h"

ADTF_FILTER_PLUGIN("Image Save Filter", OID_ADTF_IMAGE_SAVE_FILTER, ImageSaveFilter);

static int imageCount = 0;


namespace patch
{
	template < typename T > std::string to_string(const T& n)
	{
		std::ostringstream stm;
		stm << n;
		return stm.str();
	}
}


// -------------------------------------------------------------------------------------------------
ImageSaveFilter::ImageSaveFilter(const tChar* __info) {
// -------------------------------------------------------------------------------------------------
  cMemoryBlock::MemSet(&output_format_, 0, sizeof(output_format_));
  cMemoryBlock::MemSet(&input_format_, 0, sizeof(output_format_));
  
  //SetPropertyString("path", "/home/Desktop/image.png");
}

// -------------------------------------------------------------------------------------------------
ImageSaveFilter::~ImageSaveFilter() {
// -------------------------------------------------------------------------------------------------

}

// -------------------------------------------------------------------------------------------------
tResult ImageSaveFilter::Init(tInitStage stage, __exception) {
// -------------------------------------------------------------------------------------------------
  RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr));
  
	if (stage == StageFirst) {
		// Video input.
		RETURN_IF_FAILED(video_input_pin_.Create("Video_Input", IPin::PD_Input,
		  static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&video_input_pin_));

    // Video output
    RETURN_IF_FAILED(video_output_pin_.Create("Video_Output", IPin::PD_Output, NULL));
    RETURN_IF_FAILED(RegisterPin(&video_output_pin_));

    height_ = -1;
    width_ = -1;
	} else if (stage == StageGraphReady) {
	  cObjectPtr<IMediaType> type;
	  RETURN_IF_FAILED(video_input_pin_.GetMediaType(&type));
	  
	  cObjectPtr<IMediaTypeVideo> type_video;
	  RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**) &type_video));
	  
	  const tBitmapFormat* format = type_video->GetFormat();
	  if (format == NULL) RETURN_ERROR(ERR_NOT_SUPPORTED);
	  
    cMemoryBlock::MemCopy(&input_format_, format, sizeof(tBitmapFormat));
	  cMemoryBlock::MemCopy(&output_format_, format, sizeof(tBitmapFormat));
	  
    video_output_pin_.SetFormat(&output_format_, NULL);
	}
	
	RETURN_NOERROR;
}

// -------------------------------------------------------------------------------------------------
tResult ImageSaveFilter::Shutdown(tInitStage stage, __exception) {
// -------------------------------------------------------------------------------------------------
	return cFilter::Shutdown(stage,__exception_ptr);
}

// -------------------------------------------------------------------------------------------------
tResult ImageSaveFilter::OnPinEvent(IPin* source, tInt event_code, tInt param1, tInt param2,
    IMediaSample* media_sample) {
// -------------------------------------------------------------------------------------------------
	RETURN_IF_POINTER_NULL(source);
	RETURN_IF_POINTER_NULL(media_sample);
	
	if (event_code == IPinEventSink::PE_MediaSampleReceived) {
		if (source == &video_input_pin_) {
			if (height_ == -1 || width_ == -1) setHeightAndWidth();

      processImage(media_sample);
		}
	}
		
	RETURN_NOERROR;
}

// -------------------------------------------------------------------------------------------------
tResult ImageSaveFilter::processImage(IMediaSample* sample) {
// -------------------------------------------------------------------------------------------------
  RETURN_IF_POINTER_NULL(sample);
  
  cObjectPtr<IMediaSample> new_sample;
  RETURN_IF_FAILED(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &new_sample));
  
  RETURN_IF_FAILED(new_sample->AllocBuffer(output_format_.nSize));
  
  const tVoid* source_buffer;
  tVoid* dest_buffer;
  
  if (IS_OK(sample->Lock(&source_buffer))) {
    if (IS_OK(new_sample->WriteLock(&dest_buffer))) {
      // Mem-copy von source_buffer auf dest_buffer
      memcpy((uchar*)dest_buffer, (uchar*)source_buffer, 3 * height_ * width_);
      
      Mat current_image(height_, width_, CV_8UC3, (uchar*)source_buffer);
      Mat copy(height_, width_, CV_8UC3, (uchar*)dest_buffer);

      // Save the image at the specified path
      //imwrite(GetPropertyString("path"), copy);

      // Save video as a image 
      String imgPath = "/home/aadc/Desktop/images/" + patch::to_string(imageCount++) + ".png" ;
      imwrite(imgPath, copy);
	
      
      new_sample->Unlock(dest_buffer);
    }
    
    new_sample->SetTime(sample->GetTime());
    sample->Unlock(source_buffer);
  }
  
  RETURN_IF_FAILED(video_output_pin_.Transmit(new_sample));
  
  RETURN_NOERROR;
}

// -------------------------------------------------------------------------------------------------
tResult ImageSaveFilter::setHeightAndWidth() {
// -------------------------------------------------------------------------------------------------
	// Retrieve the input type
	cObjectPtr<IMediaType> type;
	RETURN_IF_FAILED(video_input_pin_.GetMediaType(&type));
	
	// Retrieve the video type based on the input type
	cObjectPtr<IMediaTypeVideo> video_type;
	RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&video_type));
	
	// Retrieve height and width values from the video type
	height_ = video_type->GetFormat()->nHeight;
	width_ = video_type->GetFormat()->nWidth;
	
	RETURN_NOERROR;
}
