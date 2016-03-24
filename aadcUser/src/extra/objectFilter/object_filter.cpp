#include "stdafx.h"
#include "object_filter.h"
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cvBlob/cvblob.h"
#include "../util/point_transformer.h"
#include "../util/vector2.h"
#include <fstream>
#include <sstream>
#include <iostream>

using namespace cv;
using namespace cvb;

ADTF_FILTER_PLUGIN("DaedALUs Object Filter", OID_ADTF_OBJECT_FILTER, ObjectFilter);

// -------------------------------------------------------------------------------------------------
ObjectFilter::ObjectFilter(const tChar* __info) {
// -------------------------------------------------------------------------------------------------
  cMemoryBlock::MemSet(&input_format_, 0, sizeof(input_format_));
  cMemoryBlock::MemSet(&output_format_, 0, sizeof(output_format_));
  
  SetPropertyStr("basePath", "/home/aadc/Desktop/images/object/base_ext_320.png");
  SetPropertyStr("basePathExt", "/home/aadc/Desktop/images/object/base_ext_320.png");
  SetPropertyInt("diffThreshold", 30);
  SetPropertyInt("whiteThreshold", 180);
  SetPropertyFloat("scaleWidth", 1);
  SetPropertyFloat("scaleHeight", 1);
  SetPropertyInt("offsetHor", 0);
  SetPropertyInt("minBlobSize", 500);
  SetPropertyInt("maxBlobSize", 1000000);
  SetPropertyInt("horizon_base" , 292); // 227@base; 292@base_ext_620; 340@base_ext_720; 350@base_720;
  SetPropertyInt("horizon_source" , 221); //change whenever camera position changes
  
}

// -------------------------------------------------------------------------------------------------
ObjectFilter::~ObjectFilter() {
// -------------------------------------------------------------------------------------------------

}

// -------------------------------------------------------------------------------------------------
tResult ObjectFilter::Init(tInitStage stage, __exception) {
// -------------------------------------------------------------------------------------------------
  RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr));
  
	if (stage == StageFirst) {
		// Create and register the video input pin
		RETURN_IF_FAILED(video_input_pin_.Create("Video_Input", IPin::PD_Input,
		  static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&video_input_pin_));
    
    // Create and register the video output pin
    RETURN_IF_FAILED(video_output_pin_.Create("Video_Output", IPin::PD_Output, NULL));
    RETURN_IF_FAILED(RegisterPin(&video_output_pin_));
    
    // Set up the media description manager object for object output
    cObjectPtr<IMediaDescriptionManager> description_manager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,
      IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**) &description_manager, __exception_ptr));
    
    // Create the output stream description for object data output
    tChar const * output_stream_type = description_manager->GetMediaDescription("ObjectArray");
    RETURN_IF_POINTER_NULL(output_stream_type);
    
    cObjectPtr<IMediaType> output_type_signal_value = new cMediaType(0, 0, 0, "ObjectArray",
      output_stream_type, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
      
    RETURN_IF_FAILED(output_type_signal_value->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION,
      (tVoid**) &object_data_description_));
      
    // Create and register the object data output pin
    RETURN_IF_FAILED(object_output_pin_.Create("objects", output_type_signal_value, NULL));
    RETURN_IF_FAILED(RegisterPin(&object_output_pin_));
    
    RETURN_IF_FAILED(mapped_object_output_pin_.Create("mapped_objects", output_type_signal_value, NULL));
    RETURN_IF_FAILED(RegisterPin(&mapped_object_output_pin_));
	}
	
	else if (stage == StageGraphReady) {
	  cObjectPtr<IMediaType> type;
	  RETURN_IF_FAILED(video_input_pin_.GetMediaType(&type));
	  
	  cObjectPtr<IMediaTypeVideo> type_video;
	  RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**) &type_video));
	  
	  const tBitmapFormat* format = type_video->GetFormat();
	  tBitmapFormat output_format;
	  output_format.nWidth = 640;
	  output_format.nHeight = 480;
	  output_format.nBitsPerPixel = 24;
	  output_format.nPixelFormat = 45;
	  output_format.nBytesPerLine = 1920;
	  output_format.nSize = 921600;
	  output_format.nPaletteSize = 0;
	  if (format == NULL) RETURN_ERROR(ERR_NOT_SUPPORTED);
	  
    cMemoryBlock::MemCopy(&input_format_, format, sizeof(tBitmapFormat));
	 cMemoryBlock::MemCopy(&output_format_, &output_format, sizeof(tBitmapFormat));
	  
    video_output_pin_.SetFormat(&output_format_, NULL);
	}
	
	RETURN_NOERROR;
}

// -------------------------------------------------------------------------------------------------
tResult ObjectFilter::Shutdown(tInitStage stage, __exception) {
// -------------------------------------------------------------------------------------------------
	return cFilter::Shutdown(stage,__exception_ptr);
}

// -------------------------------------------------------------------------------------------------
tResult ObjectFilter::OnPinEvent(IPin* source, tInt event_code, tInt param1, tInt param2,
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
tResult ObjectFilter::processImage(IMediaSample* sample) {
// -------------------------------------------------------------------------------------------------
  // Check if the sample is valid
  RETURN_IF_POINTER_NULL(sample);
  
  // Create the new image sample to transmit at the end
  cObjectPtr<IMediaSample> image_sample;
  RETURN_IF_FAILED(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &image_sample));
  RETURN_IF_FAILED(image_sample->AllocBuffer(output_format_.nSize));
  
  // Initialize the data buffers
  const tVoid* source_buffer;
  tVoid* dest_buffer;
  
  std::vector<Object> objects;
  std::vector<Object> mapped_objects;
  
  if (IS_OK(sample->Lock(&source_buffer))) {
    if (IS_OK(image_sample->WriteLock(&dest_buffer))) {
      int source_width = input_format_.nWidth;
      int source_height = input_format_.nHeight;

      // Create the source image matrix    
      Mat source_image(source_height, source_width, CV_8UC2, (uchar*)source_buffer);
      
      // Retrieve the actual depth image
      Mat source_channels[2];
      split(source_image, source_channels);
      Mat depth_image = source_channels[1];
      
      // Retrieve the base image
		int horizon_base = GetPropertyInt("horizon_base");
		int horizon_source = GetPropertyInt("horizon_source");
    Mat base_image_extended = imread(GetPropertyStr("basePathExt"), 0);
    Mat base_image = base_image_extended( Rect(0, horizon_base - horizon_source, 320, 240)); 
      
      //Mat base_image = imread(GetPropertyStr("basePath"), 0);
      int base_threshold = GetPropertyInt("diffThreshold");
      int white_threshold = GetPropertyInt("whiteThreshold");
      
      for (int i = 0; i < depth_image.rows; i++) {
        for (int j = 0; j < depth_image.cols; j++) {
          // Merge white and black noise and substract from base  
          if (depth_image.at<uchar>(i,j) >= white_threshold) depth_image.at<uchar>(i,j) = 0;
          
          // Substract the base image from the actual image
          int grey_diff = depth_image.at<uchar>(i,j) - base_image.at<uchar>(i,j);
          if (depth_image.at<uchar>(i,j) == 0 || abs(grey_diff) < base_threshold) {
            depth_image.at<uchar>(i,j) = 0;
          }/* else if (i >= 113 && i <= 130) {
            depth_image.at<uchar>(i,j) = depth_image.at<uchar>(i-1, j);
          }*/
        }
      }
      
      // Create objects used for blob detection
      Mat blob_image;
      cvtColor(depth_image, blob_image, COLOR_GRAY2BGR);
      CvBlobs blobs;
      IplImage *blob_label = cvCreateImage(cvSize(depth_image.cols, depth_image.rows), IPL_DEPTH_LABEL, 1);
      IplImage ipl_depth_image = depth_image;
      IplImage ipl_blob_image = blob_image;

      cvLabel(&ipl_depth_image, blob_label, blobs);
      cvFilterByArea(blobs, GetPropertyInt("minBlobSize"), GetPropertyInt("maxBlobSize"));
      
      for (CvBlobs::iterator i = blobs.begin(); i != blobs.end(); i++) {
        // Retrieve the blob data
        int minx = i->second->minx;
        int miny = i->second->miny;
        //int maxx = i->second->maxx;
        //int maxy = i->second->maxy;
        int width = i->second->maxx - i->second->minx;
        int height = i->second->maxy - i->second->miny;
        
          
        Object cur(
          2 * minx - GetPropertyInt("OffsetHor"), 2 * miny,
            2 * width * GetPropertyFloat("ScaleWidth"), 2* height * GetPropertyFloat("ScaleHeight"),
              2 * source_width, 2 * source_height);
        objects.push_back(cur);
        
        if (cur.get_relative_height() <= cur.get_relative_width() * 1.2) {
          Vector2 cur_origin(cur.get_absolute_x(), cur.get_absolute_y() + cur.get_absolute_height());
          Vector2 cur_max(cur.get_absolute_x() + cur.get_absolute_width(), cur.get_absolute_y() + cur.get_absolute_height());
          Vector2 cur_source(cur.get_absolute_x() + cur.get_absolute_width(), cur.get_absolute_y() + cur.get_absolute_height());
          // transformer muss durch ipm transformation ersetzt werden
          cur_origin = PointTransformer::map_to_aerial_view(cur_origin);
          cur_max = PointTransformer::map_to_aerial_view(cur_max);
          cur_source = PointTransformer::map_to_aerial_view(cur_source);
          
          int obj_width = cur_max.get_x() - cur_origin.get_x();
          int obj_height = cur_max.get_y() - cur_max.get_y();
          Object cur_mapped(cur_origin.get_x(), cur_origin.get_y(), obj_width, obj_height, cur_source.get_y(), cur_source.get_x());
          mapped_objects.push_back(cur_mapped);
       }
      }
 
      // Render the blobs into the image to be transmitted
      cvRenderBlobs(blob_label, blobs, &ipl_blob_image, &ipl_blob_image);
      
      // Copy the blobbed image into the destination buffer
      int output_height = output_format_.nHeight;
      int output_width = output_format_.nWidth;
      resize(blob_image, blob_image, Size(output_width, output_height));
      memcpy((uchar*)dest_buffer, (uchar*)blob_image.data, 3 * output_height * output_width);
      
      // Release the images used for blobbing
      cvReleaseImage(&blob_label);
      cvReleaseBlobs(blobs);
      
      image_sample->Unlock(dest_buffer);
    }
    
    image_sample->SetTime(sample->GetTime());
    sample->Unlock(source_buffer);
  }
  
  // Transmit the blobs via the object list output pin
  transmitObjects(objects, object_output_pin_);
  transmitObjects(mapped_objects, mapped_object_output_pin_);
  
  RETURN_IF_FAILED(video_output_pin_.Transmit(image_sample));
  
  RETURN_NOERROR;
}

// -------------------------------------------------------------------------------------------------
tResult ObjectFilter::transmitObjects(std::vector<Object> const & objects, cOutputPin & output_pin) {
// -------------------------------------------------------------------------------------------------
  cObjectPtr<IMediaSample> objects_sample;
  RETURN_IF_FAILED(AllocMediaSample(&objects_sample));

  RETURN_IF_FAILED(objects_sample->AllocBuffer(sizeof(tUInt32) + sizeof(Object) * objects.size()));

  tUInt32* dest_buffer = NULL;
  RETURN_IF_FAILED(objects_sample->WriteLock((tVoid**)&dest_buffer));
  
  (*dest_buffer) = (tUInt32)objects.size();
  dest_buffer++;

  cMemoryBlock::MemCopy(dest_buffer, &(objects[0]), sizeof(Object) * objects.size());
  
  RETURN_IF_FAILED(objects_sample->Unlock((tVoid*)dest_buffer));
	output_pin.Transmit(objects_sample);
	RETURN_NOERROR;
}
