/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved. TEAM DAEDALUS
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


Summary:

1) Filter for detecting parking area from the image

**********************************************************************
* $Author:: spiesra $  $Date:: 10-Feb-2016 08:29:07#$ $Rev:: 35003   $
* Updated By:: Amanullah Tariq 
**********************************************************************/


#include "stdafx.h"
#include "ParkingType.h"
#include "parkingspot_detection.h"

ADTF_FILTER_PLUGIN("DAEDALUS Parkingspot Detection", OID_ADTF_PARKINGSPOT_DETECTION, cParkingspotDetection)

#define AF_PROP_SHOW_LOG "Common::Show Log"
#define AF_PROP_DIST_FACTOR "Common::Distance Factor"
const int i =0;
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

cParkingspotDetection::cParkingspotDetection(const tChar* __info):cFilter(__info)
{
	/****************** Start of Debug ***************/
	SetPropertyBool(AF_PROP_SHOW_LOG,tTrue);
	SetPropertyStr(AF_PROP_SHOW_LOG NSSUBPROP_DESCRIPTION, "If true show log values");

	/****************** Distance Factor ***************/
	SetPropertyFloat(AF_PROP_DIST_FACTOR,0.16536);
	SetPropertyStr(AF_PROP_DIST_FACTOR NSSUBPROP_DESCRIPTION, "Factor to increase the return distance");

}

cParkingspotDetection::~cParkingspotDetection() {
// -------------------------------------------------------------------------------------------------

}

tResult cParkingspotDetection::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
  
	if (eStage == StageFirst) 
	{
		CreateInputPins(__exception_ptr);
		CreateOutputPins(__exception_ptr);
	}
	else if (eStage == StageNormal)
    {
		isFirstFrame = true;
		isEnable = false;
		DriverNextAction = 1; 
		DriverModuleBufferReaderFlag = false;
		ReadProperties();
	}
  
	RETURN_NOERROR;
}

tResult cParkingspotDetection::Shutdown(tInitStage estage, __exception) {

  if (estage == StageGraphReady)
    {
    }
    else if (estage == StageNormal)
    {
    }
    else if (estage == StageFirst)
    {
    }

  return cFilter::Shutdown(estage,__exception_ptr);
}

tResult cParkingspotDetection::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
  RETURN_IF_POINTER_NULL(pMediaSample);
  RETURN_IF_POINTER_NULL(pSource);

  if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
  {
	  tTimeStamp InputTimeStamp;
      InputTimeStamp = pMediaSample->GetTime();

	  if (pSource == &VideoInputPin && isEnable)
	   {
		   if (isFirstFrame)
            {        
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(VideoInputPin.GetMediaType(&pType));
                cObjectPtr<IMediaTypeVideo> pTypeVideo;
                RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
                const tBitmapFormat* pFormat = pTypeVideo->GetFormat();                                
                if (pFormat == NULL)
                {
                    LOG_ERROR("No Bitmap information found on pin \"input\"");
                    RETURN_ERROR(ERR_NOT_SUPPORTED);
                }
                m_sInputFormat.nPixelFormat = pFormat->nPixelFormat;
                m_sInputFormat.nWidth = pFormat->nWidth;
                m_sInputFormat.nHeight =  pFormat->nHeight;
                m_sInputFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
                m_sInputFormat.nBytesPerLine = pFormat->nBytesPerLine;
                m_sInputFormat.nSize = pFormat->nSize;
                m_sInputFormat.nPaletteSize = pFormat->nPaletteSize;
				isFirstFrame = false;
            }
            ProcessImage(pMediaSample, InputTimeStamp);
            
         
        }
	  else if(pSource == &DriverActionPin)
	  {
		   tUInt32 DriverActionTime = 0;
            tBool DriverIsProcessed = tFalse;
            {   // focus for sample read lock
                    __adtf_sample_read_lock_mediadescription(m_pDescriptionDecisionInput,pMediaSample,pCoderInput);       
                    if(!DriverModuleBufferReaderFlag)
                    {
                        pCoderInput->GetID("i32Value", DriverNextActionBuffer);
                        pCoderInput->GetID("ui32Timestamp",DriverTimeBuffer);
                        pCoderInput->GetID("bIsExecuted",DriverIsProcessedBuffer);
                        DriverModuleBufferReaderFlag = tTrue;
                    }  
                    // get the values from DriverNextAction
                    pCoderInput->Get(DriverNextActionBuffer, (tVoid*)&DriverNextAction);
                    pCoderInput->Get(DriverTimeBuffer, (tVoid*)&DriverActionTime);
                    pCoderInput->Get(DriverIsProcessedBuffer, (tVoid*)&DriverIsProcessed);       
			}
			if (m_bShowLog)   
			{
                    LOG_ERROR(cString::Format("ParkingSpot Deteciton: DriverModule Data: ID -> %d time: %d isProcessed: %d",DriverNextAction, DriverActionTime,DriverIsProcessed));
			}
	  }
	  else if(pSource == &StartPin)
	  {
		  {   // focus for sample read lock
			  __adtf_sample_read_lock_mediadescription(m_pCoderDescBool,pMediaSample,pCoder);
			  pCoder->Get("bValue", (tVoid*)&isEnable);        
		  }
		  if (m_bShowLog)   
		  {
			  LOG_ERROR(cString::Format("ParkingSpot Deteciton: Start:%d",isEnable));
		  }
		 
		  
	  }
  }
      RETURN_NOERROR;
}

/*************** Custom Method ********************/

tVoid cParkingspotDetection::ReadProperties()
{
	/********************* Properties for Show Log **************************/
	m_bShowLog = GetPropertyBool(AF_PROP_SHOW_LOG);
	distFactor = GetPropertyFloat(AF_PROP_DIST_FACTOR);
}

tResult cParkingspotDetection::CreateInputPins(__exception)
{
	cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
  
	// Media Description Bool
    tChar const * strDescSignalBoolValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalBoolValue);
    cObjectPtr<IMediaType> pTypeSignalBoolValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalBoolValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(pTypeSignalBoolValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescBool));

	// Media Description ID 
	tChar const * strDescIdValue = pDescManager->GetMediaDescription("tIdValue");    
    RETURN_IF_POINTER_NULL(strDescIdValue);
    cObjectPtr<IMediaType> pTypeIdValue = new cMediaType(0, 0, 0, "tIdValue", strDescIdValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
	RETURN_IF_FAILED(pTypeIdValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionDecisionInput)); 

	 //Input Pin from Camera
    RETURN_IF_FAILED(VideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&VideoInputPin)); 
      
     // Action Pin from Jury
    RETURN_IF_FAILED(DriverActionPin.Create("Action",pTypeIdValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&DriverActionPin));

	 // Start Pin to enable the filter
    RETURN_IF_FAILED(StartPin.Create("Start",pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&StartPin));

	RETURN_NOERROR;
}

tResult cParkingspotDetection::CreateOutputPins(__exception)
{
	cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

	// Media Description Signal
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);        
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal)); 

	 //Acceleration Output
    RETURN_IF_FAILED(ParkingInfoPin.Create("ParkingInfo", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&ParkingInfoPin));
    

	RETURN_NOERROR;
}

tResult cParkingspotDetection::ProcessImage(IMediaSample* pSample, tTimeStamp tsInputTime)
{
	RETURN_IF_POINTER_NULL(pSample);
	const tVoid* l_pSrcBuffer;
	IplImage* oImg = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
	RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
	oImg->imageData = (char*)l_pSrcBuffer;
	Mat image(cvarrToMat(oImg));
	cvReleaseImage(&oImg);
	pSample->Unlock(l_pSrcBuffer);
	inputImage = image.clone();
	    if (isEnable)
		  	GetParkingInfo(inputImage);
	 String imgPath = "/home/aadc/Desktop/images/" + patch::to_string(imageCount++) + ".png" ;
      imwrite(imgPath, inputImage);
	
	//isEnable = false;
	RETURN_NOERROR;
}

tVoid cParkingspotDetection::GetParkingInfo(Mat inputImage)
{
	if (m_bShowLog)   
	{
		LOG_ERROR(cString::Format("GetParkingInfo: Start"));
	}
	Mat outputImg;
	 if (inputImage.data)
	{
		GenerateIPM(inputImage, outputImg);
		ParkingType parkingType;
		bool result = false;
		float distance = 0.0;
		float line_distance = 0.0;
		if(DriverNextAction == PARALLEL_PARKING)
		{
			if (m_bShowLog)   
			{
				LOG_ERROR(cString::Format("GetParkingInfo: PARALLEL_PARKING"));
			}
			parkingType.GetParkingInfo(outputImg,true,result ,line_distance,distance );
			if (m_bShowLog)   
			{
				LOG_ERROR(cString::Format("GetParkingInfo: Result: %d",result ));
			}
			if (result)
				{
					TransmitParkingInfo(distance,m_tsArduinoTime);
					isEnable = false;
				}
			isEnable = false;
		}
		else if(DriverNextAction == CROSS_PARKING)
		{
			if (m_bShowLog)   
			{
				LOG_ERROR(cString::Format("GetParkingInfo: CROSS_PARKING"));
			}
			parkingType.GetParkingInfo(outputImg,false,result,line_distance ,distance );
			if (m_bShowLog)   
			{
				LOG_ERROR(cString::Format("GetParkingInfo: Result: %d",result ));
			}
			if (result)
			{
				TransmitParkingInfo(distance,m_tsArduinoTime);
				isEnable = false;
			}
				
		}else{
			LOG_ERROR(cString::Format("Wrong Parking Type"));
		}
	}
	else
	LOG_ERROR(cString::Format("Image Not Found Error"));

}

tVoid cParkingspotDetection::GenerateIPM(Mat inputImg, Mat &outputImg)
{
	vector<Point2f> origPoints;
	origPoints.push_back(Point2f(- 1000, HEIGHT));
	origPoints.push_back(Point2f(WIDTH + 1000, HEIGHT));
	origPoints.push_back(Point2f(WIDTH, 180));
	origPoints.push_back(Point2f(0, 180));

	// The 4-points correspondences in the destination image
	vector<Point2f> dstPoints;
	dstPoints.push_back(Point2f(0, HEIGHT));
	dstPoints.push_back(Point2f(WIDTH, HEIGHT));
	dstPoints.push_back(Point2f(WIDTH, 0));
	dstPoints.push_back(Point2f(0, 0));

	// IPM object
	IPM ipm(Size(WIDTH, HEIGHT), Size(WIDTH, HEIGHT), origPoints, dstPoints);
	// ipm.drawPoints(origPoints, inputImg);

	Mat inputImgGray;
	// Color Conversion
	if (inputImg.channels() == 3)
		cvtColor(inputImg, inputImgGray, CV_BGR2GRAY);
	else
		inputImg.copyTo(inputImgGray);

	ipm.applyHomography(inputImg, outputImg);
	//ipm.drawPoints(origPoints, inputImg);
}

/********************* Transmit Methods *****************************/
tResult cParkingspotDetection::TransmitParkingInfo(tFloat32 value, tTimeStamp tsInputTime)
{
	if (m_bShowLog)   
	{
		LOG_ERROR(cString::Format("TransmitParkingInfo: Start"));
	}
	value = value * distFactor;
     __synchronized_obj(m_oParkingInfoCritSection);

    //create new media sample
    cObjectPtr<IMediaSample> pSampleParkingInfo;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSampleParkingInfo));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pSampleParkingInfo->AllocBuffer(nSize));
        
    {
          __adtf_sample_write_lock_mediadescription(m_pCoderDescSignal, pSampleParkingInfo, pCoderOutput);
 
          pCoderOutput->Set("f32Value", (tVoid*)&value);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
          
     pSampleParkingInfo->SetTime(tsInputTime);
     RETURN_IF_FAILED(ParkingInfoPin.Transmit(pSampleParkingInfo));

	 if (m_bShowLog)   
	{
		LOG_ERROR(cString::Format("TransmitParkingInfo: Ended"));
	}

     RETURN_NOERROR;
}

