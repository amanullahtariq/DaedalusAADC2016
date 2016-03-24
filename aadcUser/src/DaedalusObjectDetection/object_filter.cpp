#include "stdafx.h"
#include "object_filter.h"
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cvBlob/cvblob.h"
#include "../extra/util/point_transformer.h"
#include "../extra/util/vector2.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include "daedalus_emergency_constant.h"

using namespace cv;
using namespace cvb;

ADTF_FILTER_PLUGIN("Daedalus Object Detection", OID_ADTF_OBJECT_DETECTION, ObjectFilter);

ObjectFilter::ObjectFilter(const tChar* __info) : cFilter(__info){
  
  
  SetAllProperties();
  
}

// -------------------------------------------------------------------------------------------------
ObjectFilter::~ObjectFilter() {
// -------------------------------------------------------------------------------------------------

}

// -------------------------------------------------------------------------------------------------
tResult ObjectFilter::Init(tInitStage stage, __exception) {
// -------------------------------------------------------------------------------------------------
  RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))
  
  if (stage == StageFirst)
  {
    CreateInputPins(__exception_ptr);
    CreateOutputPins(__exception_ptr);
  }
  else if (stage == StageNormal)
  {
    GetAllProperties(); 
  }
  else if (stage == StageGraphReady) {
    cObjectPtr<IMediaType> type;
    RETURN_IF_FAILED(video_input_pin_.GetMediaType(&type));
    
    cObjectPtr<IMediaTypeVideo> type_video;
    RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**) &type_video));
    
    const tBitmapFormat* format = type_video->GetFormat();
    
    if (format == NULL) RETURN_ERROR(ERR_NOT_SUPPORTED);
    
    cMemoryBlock::MemCopy(&input_format_, format, sizeof(tBitmapFormat));
  }
  
  RETURN_NOERROR;
}

tResult ObjectFilter::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult ObjectFilter::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult ObjectFilter::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

// -------------------------------------------------------------------------------------------------
tResult ObjectFilter::OnPinEvent(IPin* source, tInt event_code, tInt param1, tInt param2,
    IMediaSample* media_sample) {
// -------------------------------------------------------------------------------------------------
  RETURN_IF_POINTER_NULL(source);
  RETURN_IF_POINTER_NULL(media_sample);
  
  if (event_code == IPinEventSink::PE_MediaSampleReceived) {
    if (source == &video_input_pin_) {
     
      ProcessImage(media_sample);
     
    }
  }
    
  RETURN_NOERROR;
}

// -------------------------------------------------------------------------------------------------
tResult ObjectFilter::ProcessImage(IMediaSample* sample) {
// -------------------------------------------------------------------------------------------------
  // Check if the sample is valid
  RETURN_IF_POINTER_NULL(sample);
  // Initialize the data buffers
  const tVoid* source_buffer;
  // tVoid* dest_buffer;
  
  std::vector<Object> objects;
  // std::vector<Object> mapped_objects;
  
  RETURN_IF_FAILED(sample->Lock(&source_buffer));
// if (IS_OK(image_sample->WriteLock(&dest_buffer))) {
  int source_width = input_format_.nWidth;
  int source_height = input_format_.nHeight;
  // Create the source image matrix    
  Mat source_image(source_height, source_width, CV_8UC2, (uchar*)source_buffer);
  Mat AnotherSource_image =  source_image.clone();
  sample->Unlock(source_buffer);
  // Retrieve the actual depth image
  Mat source_channels[2];
  split(AnotherSource_image, source_channels);
  Mat depth_image = source_channels[1];
  Mat base_image_extended = imread(GetPropertyStr(EF_PROP_BASE_PATH_EXT), 0);
  Mat base_image = base_image_extended( Rect(0, horizon_base - horizon_source, 320, 240)); 
  
  for (int i = 0; i < depth_image.rows; i++) {
    for (int j = 0; j < depth_image.cols; j++) {
      // Merge white and black noise and substract from base  
      if (depth_image.at<uchar>(i,j) >= white_threshold) depth_image.at<uchar>(i,j) = 0;
      
      // Substract the base image from the actual image
      int grey_diff = depth_image.at<uchar>(i,j) - base_image.at<uchar>(i,j);
      if (depth_image.at<uchar>(i,j) == 0 || abs(grey_diff) < base_threshold) {
        depth_image.at<uchar>(i,j) = 0;
      }
    }
  }
  
  // Create objects used for blob detection
  Mat blob_image;
  cvtColor(depth_image, blob_image, COLOR_GRAY2BGR);
  CvBlobs blobs;
  IplImage *blob_label = cvCreateImage(cvSize(depth_image.cols, depth_image.rows), IPL_DEPTH_LABEL, 1);
  IplImage ipl_depth_image = depth_image;

  cvLabel(&ipl_depth_image, blob_label, blobs);
  cvFilterByArea(blobs, min_blob_size,max_blob_size);
  
  for (CvBlobs::iterator i = blobs.begin(); i != blobs.end(); i++) {
    // Retrieve the blob data
    int minx = i->second->minx;
    int miny = i->second->miny;
    int width = i->second->maxx - i->second->minx;
    int height = i->second->maxy - i->second->miny;
    
      
    Object cur(
      2 * minx - offset_hor, 2 * miny,
        2 * width * scale_height, 2* height * scale_width, 2 * source_width, 2 * source_height);
    objects.push_back(cur);
  }
  cvReleaseImage(&blob_label);
  cvReleaseBlobs(blobs);
  
  // Transmit the blobs via the object list output pin
  TransmitObjects(objects);
  
  RETURN_NOERROR;
}

// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------
tResult ObjectFilter::TransmitObjects(std::vector<Object> const & objects) 
{
  // __synchronized_obj(TransmitObjectsCricSection);
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
  object_output_pin_.Transmit(objects_sample);

  RETURN_NOERROR;
}
tResult ObjectFilter::CreateInputPins(__exception)
{ 
   //depth image
  RETURN_IF_FAILED(video_input_pin_.Create("DepthImage", adtf::IPin::PD_Input  , static_cast<IPinEventSink*> (this)));
  RETURN_IF_FAILED(RegisterPin(&video_input_pin_));
  RETURN_NOERROR;
}

tResult ObjectFilter::CreateOutputPins(__exception)
{
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
 
    RETURN_NOERROR;
}

tVoid ObjectFilter::Initialize()
{
  cMemoryBlock::MemSet(&input_format_, 0, sizeof(input_format_));
}

tVoid ObjectFilter::SetAllProperties()
{   
    SetPropertyStr(EF_PROP_BASE_PATH, "/home/aadc/Desktop/images/object/base.png");
    SetPropertyStr(EF_PROP_BASE_PATH_EXT, "/home/aadc/Desktop/images/object/base320_310.png");
    SetPropertyInt(EF_PROP_DIFF_THRESHOLD, 30);
    SetPropertyInt(EF_PROP_WHITE_THRESHOLD, 180);
    SetPropertyFloat(EF_PROP_SCALE_WIDTH, 1);
    SetPropertyFloat(EF_PROP_SCALE_HEIGHT, 1);
    SetPropertyInt(EF_PROP_OFFSET_HOR, 0);
    SetPropertyInt(EF_PROP_MIN_BLOB_SIZE, 500);
    SetPropertyInt(EF_PROP_MAX_BLOB_SIZE, 1000000);
    SetPropertyInt(EF_PROP_HORIZON_BASE , 146); // 227@base; 292@base_ext_620; 340@base_ext_720; 350@base_720;
    SetPropertyInt(EF_PROP_HORIZON_SOURCE , 110); //change whenever camera position changes
    SetPropertyBool(EF_LOG_MODE, true);

}

tVoid ObjectFilter::GetAllProperties()
{
    
    white_threshold = GetPropertyInt(EF_PROP_WHITE_THRESHOLD);
    base_threshold = GetPropertyInt(EF_PROP_DIFF_THRESHOLD);
    scale_width = GetPropertyFloat(EF_PROP_SCALE_WIDTH);
    scale_height = GetPropertyFloat(EF_PROP_SCALE_HEIGHT);
    offset_hor = GetPropertyInt(EF_PROP_HORIZON_BASE);
    min_blob_size = GetPropertyInt(EF_PROP_MIN_BLOB_SIZE);
    max_blob_size = GetPropertyInt(EF_PROP_MAX_BLOB_SIZE);
    horizon_base = GetPropertyInt(EF_PROP_HORIZON_BASE);
    horizon_source = GetPropertyInt(EF_PROP_HORIZON_SOURCE);
    logModeEnabled=GetPropertyBool(EF_LOG_MODE);
}