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

1) Filter for performing action on the crossroads.

**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
* Updated By:: Amanullah Tariq 
**********************************************************************/

#include "stdafx.h"
#include "DAEDALUS_util.h"
#include "daedalus_action_enums.h"
#include "constant.h"
#include "IPM.h"
#include "ParkingType.h"
#include "daedalus_action.h"

ADTF_FILTER_PLUGIN("DAEDALUS Action Filter", OID_ADTF_DaedalusActionFilter, cActionFilter)

/************** Turn Left Properties *********************/
#define AF_PROP_LEFT_ANGLE "Action Filter::Left Turn::Angle::Left Angle"
#define AF_PROP_LEFT_STRAIGHT_ANGLE "Action Filter::Left Turn::Angle::Straight Angle"
#define AF_PROP_LEFT_SPEED "Action Filter::Left Turn::Speed::Left Speed"
#define AF_PROP_LEFT_STRAIGHT_SPEED "Action Filter::Left Turn::Speed::Straight Speed"

#define AF_PROP_LEFT_DURATION_FORWARD "Action Filter::Left Turn::Duration::Duration Forward"
#define AF_PROP_LEFT_DURATION_TURNING "Action Filter::Left Turn::Duration::Duration Turning"

#define AF_PROP_LEFT_STRAIGHT_DISTANCE "Action Filter::Left Turn::Distance::Straight Distance"
#define AF_PROP_LEFT_DISTANCE "Action Filter::Left Turn::Distance::Left Distance"
#define AF_PROP_LEFT_TOTAL_DISTANCE "Action Filter::Left Turn::Distance::Total Distance"

/************* Turn Right Properties *******************/
#define AF_PROP_RIGHT_ANGLE "Action Filter::Right Turn::Angle::Right Angle"
#define AF_PROP_RIGHT_STRAIGHT_ANGLE "Action Filter::Right Turn::Angle::Straight Angle"
#define AF_PROP_RIGHT_SPEED "Action Filter::Right Turn::Speed::Right Speed"
#define AF_PROP_RIGHT_STRAIGHT_SPEED "Action Filter::Right Turn::Speed::Straight Speed"

#define AF_PROP_RIGHT_DURATION_FORWARD "Action Filter::Right Turn::Duration::Duration Forward"
#define AF_PROP_RIGHT_DURATION_TURNING "Action Filter::Right Turn::Duration::Duration Turning"

#define AF_PROP_RIGHT_STRAIGHT_DISTANCE "Action Filter::Right Turn::Distance::Straight Distance"
#define AF_PROP_RIGHT_DISTANCE "Action Filter::Right Turn::Distance::Right Distance"
#define AF_PROP_RIGHT_TOTAL_DISTANCE "Action Filter::Right Turn::Distance::Total Distance"

/************* Move Straight Properties *****************/
#define AF_PROP_STRAIGHT_STRAIGHT_ANGLE "Action Filter::Straight::Straight Angle"
#define AF_PROP_STRAIGHT_SPEED "Action Filter::Straight::Straight::Speed"
#define AF_PROP_STRAIGHT_DURATION_FORWARD "Action Filter::Straight::Duration Forward"

#define AF_PROP_STRAIGHT_DISTANCE "Action Filter::Straight::Straight Distance"
#define AF_PROP_STRAIGHT_TOTAL_DISTANCE "Action Filter::Straight::Total Distance"

/************* Cross Parking Properties ****************/
#define AF_PROP_CROSS_PARKING_LEFT_ANGLE "Action Filter::Cross Parking::Angle::Forward Left Angle"
#define AF_PROP_CROSS_PARKING_STRAIGHT_ANGLE "Action Filter::Cross Parking::Angle::Forward Straight Angle"
#define AF_PROP_CROSS_PARKING_RIGHT_ANGLE "Action Filter::Cross Parking::Angle::Backward Right Angle"
#define AF_PROP_CROSS_PARKING_BACK_ANGLE "Action Filter::Cross Parking::Angle::Backward Angle"

#define AF_PROP_CROSS_PARKING_LEFT_SPEED "Action Filter::Cross Parking::Speed::Forward Left Speed"
#define AF_PROP_CROSS_PARKING_STRAIGHT_SPEED "Action Filter::Cross Parking::Speed::Straight Speed"
#define AF_PROP_CROSS_PARKING_RIGHT_SPEED "Action Filter::Cross Parking::Speed::Backward Right Speed"
#define AF_PROP_CROSS_PARKING_BACKWARD_SPEED "Action Filter::Cross Parking::Speed::Backward Speed"


#define AF_PROP_CROSS_PARKING_DURATION_FORWARD "Action Filter::Cross Parking::Duration::Duration Forward"
#define AF_PROP_CROSS_PARKING_DURATION_LEFT_TURNING "Action Filter::Cross Parking::Duration::Duration Left Turning"
#define AF_PROP_CROSS_PARKING_DURATION_RIGHT_TURNING "Action Filter::Cross Parking::Duration::Duration Right Turning"
#define AF_PROP_CROSS_PARKING_DURATION_BACKWARD "Action Filter::Cross Parking::Duration::Duration Back"

#define AF_PROP_CROSS_PARKING_STRAIGHT_DISTANCE "Action Filter::Cross Parking::Distance::Straight Distance"
#define AF_PROP_CROSS_PARKING_RIGHT_DISTANCE "Action Filter::Cross Parking::Distance::Straight Distance"
#define AF_PROP_CROSS_PARKING_TOTAL_DISTANCE "Action Filter::Cross Parking::Distance::Total Distance"

/*********** Parallel Parking Properties **************/
#define AF_PROP_PARALLEL_PARKING_FORWARD_ANGLE "Action Filter::Parallel Parking::Angle::Forward Angle"
#define AF_PROP_PARALLEL_PARKING_FORWARD_LEFT_ANGLE "Action Filter::Parallel Parking::Angle::Forward Left Angle"
#define AF_PROP_PARALLEL_PARKING_RIGHT_ANGLE "Action Filter::Parallel Parking::Angle::Backward Right Angle"
#define AF_PROP_PARALLEL_PARKING_LEFT_ANGLE "Action Filter::Parallel Parking::Angle::Backward Left Angle"
#define AF_PROP_PARALLEL_PARKING_BACKWARD_ANGLE "Action Filter::Parallel Parking::Angle::Backward Angle"
#define AF_PROP_PARALLEL_PARKING_FORWARD_LAST_ANGLE "Action Filter::Parallel Parking::Angle::Duration Forward Last Angle"

#define AF_PROP_PARALLEL_PARKING_FORWARD_SPEED "Action Filter::Parallel Parking::Speed::Forward Speed"
#define AF_PROP_PARALLEL_PARKING_FORWARD_LEFT_SPEED "Action Filter::Parallel Parking::Speed::Forward Left Speed"
#define AF_PROP_PARALLEL_PARKING_RIGHT_SPEED "Action Filter::Parallel Parking::Speed::Backward Right Speed"
#define AF_PROP_PARALLEL_PARKING_LEFT_SPEED "Action Filter::Parallel Parking::Speed::Backward Left Speed"
#define AF_PROP_PARALLEL_PARKING_BACKWARD_SPEED "Action Filter::Parallel Parking::Speed::Backward Speed"
#define AF_PROP_PARALLEL_PARKING_FORWARD_LAST_SPEED "Action Filter::Parallel Parking::Speed::Duration Forward Last Speed"

#define AF_PROP_PARALLEL_PARKING_DURATION_FORWARD "Action Filter::Parallel Parking::Duration::Duration Forward"
#define AF_PROP_PARALLEL_PARKING_DURATION_FORWARD_LEFT "Action Filter::Parallel Parking::Duration::Duration Forward Left"
#define AF_PROP_PARALLEL_PARKING_DURATION_LEFT_TURNING "Action Filter::Parallel Parking::Duration::Duration Backward Left Turning"
#define AF_PROP_PARALLEL_PARKING_DURATION_RIGHT_TURNING "Action Filter::Parallel Parking::Duration::Duration Backward  Right Turning"
#define AF_PROP_PARALLEL_PARKING_DURATION_BACKWARD "Action Filter::Parallel Parking::Duration::Duration Backward"
#define AF_PROP_PARALLEL_PARKING_DURATION_FORWARD_LAST "Action Filter::Parallel Parking::Duration::Duration Forward Last"

//#define AF_PROP_CROSS_PARKING_STRAIGHT_DISTANCE "Action Filter::Parallel Parking::Distance::Straight Distance"
//#define AF_PROP_CROSS_PARKING_RIGHT_DISTANCE "Action Filter::Parallel Parking::Distance::Straight Distance"
//#define AF_PROP_CROSS_PARKING_TOTAL_DISTANCE "Action Filter::Parallel Parking::Distance::Total Distance"

/************ Pull RIGHT Properties ******************/
#define AF_PROP_PR_RIGHT_SPEED "Action Filter::PullOver Right::Speed::Right Speed"
#define AF_PROP_PR_RIGHT_ANGLE "Action Filter::PullOver Right::Angle::Right Angle"
#define AF_PROP_PR_RIGHT_DURATION "Action Filter::PullOver Right::Duration::Right Duration"
#define AF_PROP_PR_RIGHT_STRAIGHT_SPEED "Action Filter::PullOver Right::Speed::Straight Speed"
#define AF_PROP_PR_RIGHT_STRAIGHT_ANGLE "Action Filter::PullOver Right::Angle::Straight Angle"
#define AF_PROP_PR_RIGHT_DURATION_STRAIGHT "Action Filter::PullOver Right::Duration::Straight Duration"

/************ Pull Left Properties ******************/
#define AF_PROP_PL_LEFT_SPEED "Action Filter::PullOver Left::Speed::Left Speed"
#define AF_PROP_PL_LEFT_ANGLE "Action Filter::PullOver Left::Angle::Left Angle"
#define AF_PROP_PL_LEFT_DURATION "Action Filter::PullOver Left::Duration::Left Duration"
#define AF_PROP_PL_LEFT_STRAIGHT_SPEED "Action Filter::PullOver Left::Speed::Straight Speed"
#define AF_PROP_PL_LEFT_STRAIGHT_ANGLE "Action Filter::PullOver Left::Angle::Straight Angle"
#define AF_PROP_PL_LEFT_DURATION_STRAIGHT "Action Filter::PullOver Left::Duration::Straight Duration"

/************ Pullout Left Parallel Parking Properties ******************/
#define AF_PROP_PR_PP_BACK_SPEED "Action Filter::PullOver Right Parallel::Speed::Back Speed"
#define AF_PROP_PR_PP_LEFT_SPEED "Action Filter::PullOver Right Parallel::Speed::Left Speed"
#define AF_PROP_PR_PP_RIGHT_SPEED "Action Filter::PullOver Right Parallel::Speed::Right Speed"

#define AF_PROP_PR_PP_BACK_ANGLE "Action Filter::PullOver Right Parallel::Angle::Back Angle"
#define AF_PROP_PR_PP_LEFT_ANGLE "Action Filter::PullOver Right Parallel::Angle::Left Angle"
#define AF_PROP_PR_PP_RIGHT_ANGLE "Action Filter::PullOver Right Parallel::Angle::Right Angle"

#define AF_PROP_PR_PP_BACK_DURATION "Action Filter::PullOver Right Parallel::Duration::Back Duration"
#define AF_PROP_PR_PP_LEFT_DURATION "Action Filter::PullOver Right Parallel::Duration::Left Duration"
#define AF_PROP_PR_PP_RIGHT_DURATION "Action Filter::PullOver Right Parallel::Duration::Right Duration"

/************ Pullout Right Parallel Parking Properties ******************/
#define AF_PROP_PL_PP_BACK_SPEED "Action Filter::PullOver Left Parallel::Speed::Back Speed"
#define AF_PROP_PL_PP_LEFT_SPEED "Action Filter::PullOver Left Parallel::Speed::Left Speed"
#define AF_PROP_PL_PP_FORWARD_SPEED "Action Filter::PullOver Left Parallel::Speed::Forward Speed"

#define AF_PROP_PL_PP_BACK_ANGLE "Action Filter::PullOver Left Parallel::Angle::Back Angle"
#define AF_PROP_PL_PP_LEFT_ANGLE "Action Filter::PullOver Left Parallel::Angle::Left Angle"
#define AF_PROP_PL_PP_FORWARD_ANGLE "Action Filter::PullOver Left Parallel::Angle::Forward Angle"

#define AF_PROP_PL_PP_BACK_DURATION "Action Filter::PullOver Left Parallel::Duration::Back Duration"
#define AF_PROP_PL_PP_LEFT_DURATION "Action Filter::PullOver Left Parallel::Duration::Left Duration"
#define AF_PROP_PL_PP_FORWARD_DURATION "Action Filter::PullOver Left Parallel::Duration::Forward Duration"


// define the ADTF property names to avoid errors 
#define AF_PROP_SHOW_LOG "Common::Show Log"
#define AF_INTERVAL_TIME "Common::Interval Time"
#define AF_PROP_LAST_COMMAND "Common::Last Command"

const int width = 640;
const int height = 480;
const float scale = 1.0;
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

cActionFilter::cActionFilter(const tChar* __info) : adtf::cTimeTriggeredFilter(__info)
{

	/****************** Start of Debug ***************/
	SetPropertyBool(AF_PROP_SHOW_LOG,tTrue);
	SetPropertyStr(AF_PROP_SHOW_LOG NSSUBPROP_DESCRIPTION, "If true show log values");
	/***************** End of Debug *****************/

	/****************** Select Type of Parking ***************/
	SetPropertyInt(AF_PROP_LAST_COMMAND,-1);
	SetPropertyStr(AF_PROP_LAST_COMMAND NSSUBPROP_DESCRIPTION, "Number of last parking type");
	/***************** End of Debug *****************/


	SetCrossParkingProperties();
	SetParallelParkingProperties();
	SetPullOverLeftProperties();
	SetPullOverRightProperties();
	SetPullOverLeftFromParallelParkingProperties();
	SetPullOverRightFromParallelParkingProperties();

	/*************************Start Properties for Left and right turn of the car **********************************/
	SetLeftTurnProperties();
	SetRightTurnProperties();
	SetStraightProperties();

//	SetPropertyBool(AF_PROP_TURN_RIGHT,tFalse);
//	SetPropertyStr(AF_PROP_TURN_RIGHT NSSUBPROP_DESCRIPTION,"If true then car will turn from the next crossroad"); 

	//SetPropertyFloat(AF_PROP_RIGHT_ANGLE,130.0);
	//SetPropertyBool(AF_PROP_RIGHT_ANGLE NSSUBPROP_ISCHANGEABLE,tTrue);
	//SetPropertyStr(AF_PROP_RIGHT_ANGLE NSSUBPROP_DESCRIPTION, "Angle for right turn");

	/*************************End Properties for Left and right turn of the car **********************************/
      //SetPropertyFloat(AF_STRAIGHT_ANGLE, 90.0); 
      SetPropertyFloat(AF_INTERVAL_TIME, 20000);   
	  //SetPropertyFloat(AF_PROP_SPEED, 1.0);
	  //SetPropertyFloat(AF_PROP_DF, 60);
	  //SetPropertyFloat(AF_PROP_DURATION_TURNING, 90);
}

cActionFilter::~cActionFilter()
{

}

tResult cActionFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cTimeTriggeredFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
		CreateInputPins(__exception_ptr);
		CreateOutputPins(__exception_ptr);
    }
    else if (eStage == StageNormal)
    {
		m_ui8InitCtrl = 0;
		pp_counter = 0;
		cp_counter = 0;
		total_counter = 31;
		m_bFirstFrame = true;
		DriverNextAction = -1;
		prevDriverAction = -1;
		DriverModuleBufferReaderFlag = tFalse;
		ReadProperties();
		counter_ = 0;
		active_ = false;

        RETURN_IF_FAILED(SetInterval(m_f32IntervalTime));

    }
    else if (eStage == StageGraphReady)
    {
    }

    RETURN_NOERROR;
}

tResult cActionFilter::Cycle(__exception)
{
	RunAction();
    RETURN_NOERROR;
}

tResult cActionFilter::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        tTimeStamp InputTimeStamp;
        InputTimeStamp = pMediaSample->GetTime();
		
		 if(pSource == &m_iVideoInputPin)
        {
            //Videoformat
            if (m_bFirstFrame)
            {        
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(m_iVideoInputPin.GetMediaType(&pType));
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
                m_bFirstFrame = false;
            }
            ProcessInput(pMediaSample, InputTimeStamp);
        }
        if(pSource == &m_iDriverAction)
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
			active_ = true;
			if (m_bShowLog)   
			{
                    LOG_ERROR(cString::Format("Action Fiter: DriverModule Data: ID -> %d time: %d isProcessed: %d",DriverNextAction, DriverActionTime,DriverIsProcessed));
			}
        }
    }
        RETURN_NOERROR;
}

/********************* Custom Methods *****************************/
tResult cActionFilter::ProcessInput(IMediaSample* pSample, tTimeStamp tsInputTime)
{
	if(m_ui8InitCtrl < 150)         // This loop is necessary to boot the car's controller (needs neutral signal for a certain time)
    {
		//TransmitLaneTrackingInfo(0.0f,0.0f,0.0f,tTrue,tsInputTime);
		m_ui8InitCtrl++;
    }
	else
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
	}
    RETURN_NOERROR;            
}

tResult cActionFilter::RunAction()
{
	if (!active_) RETURN_NOERROR;

	if(DriverNextAction == Driver_straight)
	{
		RunStraightScript();
		prevDriverAction = DriverNextAction;
	}
	else if (DriverNextAction == Driver_left)
	{
		RunTurnLeftScript();
		prevDriverAction = DriverNextAction;
	}
	else if (DriverNextAction == Driver_right)
	{
		RunTurnRightScript();
		prevDriverAction = DriverNextAction;
	}
	else if(DriverNextAction == Driver_parallel_parking)
	{
		RunParallelParkingScript();
		prevDriverAction = DriverNextAction;
	}
	else if(DriverNextAction == Driver_cross_parking)
	{
		RunCrossParkingScript();
		prevDriverAction = DriverNextAction;
	}
	else if(DriverNextAction == Driver_pull_out_left)
	{
		if(m_bShowLog)
		{
			LOG_ERROR(cString::Format("Pull out left : started"));
		}
		if( (pp_counter + cp_counter) >= total_counter)
		{
			LOG_ERROR(cString::Format("Runing Parking detected"));
			if(cp_counter >= pp_counter)
			{
				RunPullOverLeftScript();
			}
			else
			{
				RunPullOverLeftFromParallelParkingScript();
			}
			
		}
		else 
		{
			if(prevDriverAction == Driver_cross_parking)
			{
				RunPullOverLeftScript();
			}
			else if(prevDriverAction == Driver_parallel_parking)
			{
				RunPullOverLeftFromParallelParkingScript();
			}
			else if (inputImage.data && CrossedParking() == CROSS_PARKING)
			{
				cp_counter++;
				if(m_bShowLog)
				{
					LOG_ERROR(cString::Format("Crossed Parking detected"));
				}
			}
		}
			else if (inputImage.data)
			{
				pp_counter++;
				if(m_bShowLog)
				{
					LOG_ERROR(cString::Format("Parallel Parking detected"));
				}
			}
		}
		
		prevDriverAction = DriverNextAction;
		
	}
	else if(DriverNextAction == Driver_pull_out_right)
	{
		if( (pp_counter + cp_counter) == total_counter)
		{
			if(cp_counter > pp_counter)
			{
				RunPullOverRightScript();
			}
			else
			{
				RunPullOverRightFromParallelParkingScript();
			}
		}
		else 
		{

			if(prevDriverAction == Driver_cross_parking)
			{
				RunPullOverRightScript();
			}
			else if(prevDriverAction == Driver_parallel_parking)
			{
				RunPullOverRightFromParallelParkingScript();
			}
			else if (inputImage.data && CrossedParking() == CROSS_PARKING)
			{
				cp_counter++;
			}
			else if (inputImage.data)
			{
				pp_counter++;
			}
		}
		
		prevDriverAction = DriverNextAction;
	}
  RETURN_NOERROR;
}

tResult cActionFilter::CreateInputPins(__exception)
{
	cObjectPtr<IMediaDescriptionManager> pDescManager;
	RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

	 // Video Input
    RETURN_IF_FAILED(m_iVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_iVideoInputPin));

	 // Reading Action to perfrom
    tChar const * strDescIdValue = pDescManager->GetMediaDescription("tIdValue");    
    RETURN_IF_POINTER_NULL(strDescIdValue);
    cObjectPtr<IMediaType> pTypeIdValue = new cMediaType(0, 0, 0, "tIdValue", strDescIdValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    
    RETURN_IF_FAILED(pTypeIdValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionDecisionInput)); 
    RETURN_IF_FAILED(m_iDriverAction.Create("Action",pTypeIdValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_iDriverAction));
	

	RETURN_NOERROR;
}

tResult cActionFilter::CreateOutputPins(__exception)
{

	cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // Media Description Signal Value
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);        
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal)); 
	
	// Media Description Signal Bool
	tChar const * strDescSignalBoolValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalBoolValue);
    cObjectPtr<IMediaType> pTypeSignalBoolValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalBoolValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(pTypeSignalBoolValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescBool));


	   //Acceleration Output
    RETURN_IF_FAILED(m_oAccelerateOutputPin.Create("Acceleration", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oAccelerateOutputPin));
    
    // Steer Angle Output pin
    RETURN_IF_FAILED(m_oSteerOutputPin.Create("Steering_Angle", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oSteerOutputPin));

	 // Left Indicator Output pin
    RETURN_IF_FAILED(m_oTurnSignalLeftOutputPin.Create("Left_Indicator", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oTurnSignalLeftOutputPin));

	// Right Indicator Output Pin
    RETURN_IF_FAILED(m_oTurnSignalRightOutputPin.Create("Right_Indicator", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oTurnSignalRightOutputPin));

	 RETURN_IF_FAILED(m_oReverseIndicatorOutputPin.Create("Rever_Indicator", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oReverseIndicatorOutputPin));

	// Action Performed Output Pin
    RETURN_IF_FAILED(m_oActionPerformed.Create("Action_Performed", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oActionPerformed));


	RETURN_NOERROR;
}

tResult cActionFilter::ReadProperties()
{

	/********************* Properties for Show Log **************************/
	m_bShowLog = GetPropertyBool(AF_PROP_SHOW_LOG);
	m_f32IntervalTime = GetPropertyInt(AF_INTERVAL_TIME);
	lastCommand = GetPropertyInt(AF_PROP_LAST_COMMAND);
	if(lastCommand != -1)
	{
		prevDriverAction = lastCommand;
	}
    RETURN_NOERROR;
}

tVoid cActionFilter::GenerateIPM(Mat inputImg, Mat &outputImg)
{
	vector<Point2f> origPoints;
	origPoints.push_back(Point2f(-100, HEIGHT));
	origPoints.push_back(Point2f(WIDTH + 100, HEIGHT));
	origPoints.push_back(Point2f(WIDTH / 2 + 180, 280));
	origPoints.push_back(Point2f(WIDTH / 2 - 220, 280));

	 //The 4-points correspondences in the destination image
	vector<Point2f> dstPoints;
	dstPoints.push_back(Point2f(0, HEIGHT));
	dstPoints.push_back(Point2f(WIDTH, HEIGHT));
	dstPoints.push_back(Point2f(WIDTH , 0));
	dstPoints.push_back(Point2f(0, 0));

	//IPM object
	IPM ipm(Size(WIDTH, HEIGHT), Size(WIDTH , HEIGHT), origPoints, dstPoints);

	Mat inputImgGray;
	// Color Conversion
	if (inputImg.channels() == 3)
		cvtColor(inputImg, inputImgGray, CV_BGR2GRAY);
	else
		inputImg.copyTo(inputImgGray);

	ipm.applyHomography(inputImg, outputImg);
	ipm.drawPoints(origPoints, inputImg);
}

int cActionFilter::CrossedParking()
{
	int result = NOT_FOUND;
	//float threshValue = 100.0;
	Mat gray, binary;
	Mat outputImg;
	
	String imgPath = "/home/aadc/Desktop/images/" + patch::to_string(imageCount++) + ".png" ;
	imwrite(imgPath, inputImage);
	
	GenerateIPM(inputImage.clone(), outputImg);
		
	//Mat image = im(Rect(0, _horizon, width, _roi_height));
	cvtColor(outputImg, gray, CV_BGR2GRAY);
	GaussianBlur(gray, gray, Size(3, 3), 0, 0);
	gray = DAEDALUS::correctGamma(gray, 0.4);

	ParkingType parkingType;
	result = parkingType.IsCrossedParking(gray);
	
	return result;
}

/********************* Transmit Methods *****************************/
tResult cActionFilter::TransmitAcceleration(tFloat32 f32Acceleration, tTimeStamp tsInputTime)
{
    __synchronized_obj(m_oTransmitAccelCritSection);

    //create new media sample
    cObjectPtr<IMediaSample> pSampleAccel;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSampleAccel));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pSampleAccel->AllocBuffer(nSize));
        
    {
          __adtf_sample_write_lock_mediadescription(m_pCoderDescSignal, pSampleAccel, pCoderOutput);
 
          pCoderOutput->Set("f32Value", (tVoid*)&f32Acceleration);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
          
     pSampleAccel->SetTime(tsInputTime);
     RETURN_IF_FAILED(m_oAccelerateOutputPin.Transmit(pSampleAccel));

     RETURN_NOERROR;
}

tResult cActionFilter::TransmitSteeringAngle(const tFloat32 f32SteeringAngle, tTimeStamp tsInputTime)
{
    __synchronized_obj(m_oTransmitSteerCritSection);
    
    //create new media sample
    cObjectPtr<IMediaSample> pNewMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));

    {
          __adtf_sample_write_lock_mediadescription(m_pCoderDescSignal, pNewMediaSample, pCoderOutput);    
 
          pCoderOutput->Set("f32Value", (tVoid*)&f32SteeringAngle);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
    
    pNewMediaSample->SetTime(tsInputTime);
    RETURN_IF_FAILED(m_oSteerOutputPin.Transmit(pNewMediaSample));

    //LOG_INFO(cString::Format("Sending SteeringAngle: %f", SteeringAngle).GetPtr());

    RETURN_NOERROR;
    
}

tResult cActionFilter::TransmitRightIndicator(const tBool bTurnRightIndicator, tTimeStamp tsInputTime)
{
    __synchronized_obj(m_oTransmitRightIndicatorCritSection);
    
    //create new media sample
    cObjectPtr<IMediaSample> pNewMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescBool->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));

    {
          __adtf_sample_write_lock_mediadescription(m_pCoderDescBool, pNewMediaSample, pCoderOutput);    
 
          pCoderOutput->Set("bValue", (tVoid*)&bTurnRightIndicator);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
              
    pNewMediaSample->SetTime(tsInputTime);
    RETURN_IF_FAILED(m_oTurnSignalRightOutputPin.Transmit(pNewMediaSample));

    RETURN_NOERROR;
    
}

tResult cActionFilter::TransmitLeftIndicator(const tBool bTurnLeftIndicator, tTimeStamp tsInputTime)
{
    __synchronized_obj(m_oTransmitLeftIndicatorCritSection);
    
    //create new media sample
    cObjectPtr<IMediaSample> pNewMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescBool->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));

    {
          __adtf_sample_write_lock_mediadescription(m_pCoderDescBool, pNewMediaSample, pCoderOutput);    
 
          pCoderOutput->Set("bValue", (tVoid*)&bTurnLeftIndicator);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
              
    pNewMediaSample->SetTime(tsInputTime);
    RETURN_IF_FAILED(m_oTurnSignalLeftOutputPin.Transmit(pNewMediaSample));

    RETURN_NOERROR;
    
}

tResult cActionFilter::TransmitActionPerformed()
{
	tBool bAction = tTrue;
	__synchronized_obj(m_oTransmitActionPerformedCritSection);
    
    //create new media sample
    cObjectPtr<IMediaSample> pNewMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescBool->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));

    {
          __adtf_sample_write_lock_mediadescription(m_pCoderDescBool, pNewMediaSample, pCoderOutput);    
 
          pCoderOutput->Set("bValue", (tVoid*)&bAction);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
              
    pNewMediaSample->SetTime(m_tsArduinoTime);
    RETURN_IF_FAILED(m_oActionPerformed.Transmit(pNewMediaSample));

    RETURN_NOERROR;
}

tResult cActionFilter::TransmitReverseIndicator(const tBool bTurnOnIndicator, tTimeStamp tsInputTime)
{
    __synchronized_obj(m_oTrasmitReverseIndicatorCritSection);
    
    //create new media sample
    cObjectPtr<IMediaSample> pNewMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescBool->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));

    {
          __adtf_sample_write_lock_mediadescription(m_pCoderDescBool, pNewMediaSample, pCoderOutput);    
 
          pCoderOutput->Set("bValue", (tVoid*)&bTurnOnIndicator);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
              
    pNewMediaSample->SetTime(tsInputTime);
    RETURN_IF_FAILED(m_oReverseIndicatorOutputPin.Transmit(pNewMediaSample));

    RETURN_NOERROR;
    
}


/********************* Scripts Methods *****************************/
tVoid cActionFilter::RunTurnLeftScript()
{
    /************ Turn Left Properties *******************/
	GetLeftTurnProperties();
	if(m_bShowLog)
	{
		LOG_ERROR(cString::Format("RunTurnLeftScript: Started"));
	}
	
	TransmitLeftIndicator(tTrue,m_tsArduinoTime);
	
	if (counter_ < lt_straightDuration)
	{
		if(m_bShowLog)
		{
			LOG_ERROR(cString::Format("RunTurnLeftScript: Moving Straight"));
		}
		TransmitAcceleration(lt_straightSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(lt_straightAngle,m_tsArduinoTime);
		counter_++;
	} 
	else if (counter_ < (lt_straightDuration + lt_leftDuration ))
	{
		if(m_bShowLog)
		{
			LOG_ERROR(cString::Format("RunTurnLeftScript: Moving Left"));
		}
		TransmitAcceleration(lt_leftSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(lt_leftAngle,m_tsArduinoTime);
		counter_++;
	} 
	else 
	{
		TransmitLeftIndicator(tFalse,m_tsArduinoTime);
		TransmitAcceleration(0.0f,m_tsArduinoTime);
		TransmitSteeringAngle(lt_straightAngle,m_tsArduinoTime);
		TransmitActionPerformed();
		counter_ = 0;
		active_ = false;
		if(m_bShowLog)
		{
			LOG_ERROR(cString::Format("RunTurnLeftScript: Started"));
		}
	}
}

tVoid cActionFilter::RunTurnRightScript()
{
    /************ Turn Right Properties *******************/
	GetRightTurnProperties();
	if(m_bShowLog)
	{
		LOG_ERROR(cString::Format("RunTurnRightScript: Start"));
	}
	TransmitRightIndicator(tTrue,m_tsArduinoTime);

	if (counter_ < rt_straightDuration )
	 {
		 if(m_bShowLog)
		{
			LOG_ERROR(cString::Format("RunTurnRightScript: Moving Straight"));
		}
		 TransmitAcceleration(rt_straightSpeed,m_tsArduinoTime);
		 TransmitSteeringAngle(rt_straightAngle,m_tsArduinoTime);
		counter_++;
	 } 
	else if (counter_ < ( rt_straightDuration + rt_rightDuration) )
	 {
		 if(m_bShowLog)
		{
			LOG_ERROR(cString::Format("RunTurnRightScript: Moving Right"));
		}
		 TransmitAcceleration(rt_rightSpeed,m_tsArduinoTime);
		 TransmitSteeringAngle(rt_rightAngle,m_tsArduinoTime);
		counter_++;
	 } 
	 else 
	 {
		  TransmitRightIndicator(tFalse,m_tsArduinoTime);
	      TransmitAcceleration(0.0f,m_tsArduinoTime);
		  TransmitSteeringAngle(rt_straightAngle,m_tsArduinoTime);
		  TransmitActionPerformed();
		  counter_ = 0;
		  active_ = false;
		  if(m_bShowLog)
		  {
			  LOG_ERROR(cString::Format("RunTurnRightScript: Ended"));
		  }
	  }

}

tVoid cActionFilter::RunStraightScript()
{
    /************ Straight Properties *******************/
	GetStraightProperties();
	if(m_bShowLog)
	{
		LOG_ERROR(cString::Format("Action Fiter: In straight "));
	}
	if (counter_ < st_straightDuration )
	{
		TransmitAcceleration(st_straightSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(st_straightAngle,m_tsArduinoTime);
		counter_++;
	 } 
     else 
     {
		 TransmitAcceleration(0.0f,m_tsArduinoTime);
		 TransmitSteeringAngle(90.0,m_tsArduinoTime);
		 TransmitActionPerformed();
		 counter_ = 0;
		 active_ = false;
	}
}

tVoid cActionFilter::RunCrossParkingScript()
{
    /************ Cross Parking Properties *******************/
	GetCrossParkingProperties();
	TransmitRightIndicator(tTrue,m_tsArduinoTime);
	if(m_bShowLog)
	{
		LOG_ERROR(cString::Format("RunCrossParkingScript: Cross Parking Started"));
	}
	if (counter_ < cp_straightDuration )
	{
		TransmitAcceleration(cp_straightSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(cp_straightAngle,m_tsArduinoTime);
		counter_++;
	} 
	else if (counter_ < (cp_straightDuration + cp_leftDuration) )
	{
		TransmitAcceleration(cp_leftSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(cp_leftAngle,m_tsArduinoTime);
		counter_++;
	}
	else if (counter_ <(cp_straightDuration + cp_leftDuration + cp_rightDuration) )
	{
		TransmitReverseIndicator(tTrue,m_tsArduinoTime);
		TransmitAcceleration(cp_rightSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(cp_rightAngle,m_tsArduinoTime);
		counter_++;
	}
	else if (counter_ < (cp_straightDuration + cp_leftDuration + cp_rightDuration + cp_backDuration))
	{
		TransmitReverseIndicator(tTrue,m_tsArduinoTime);
		TransmitAcceleration(cp_backSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(cp_backAngle,m_tsArduinoTime);
		counter_++;
	}
	else 
	{
		TransmitAcceleration(0.0f,m_tsArduinoTime);
		TransmitSteeringAngle(90.0,m_tsArduinoTime);
		TransmitReverseIndicator(tFalse,m_tsArduinoTime);
		TransmitRightIndicator(tFalse,m_tsArduinoTime);
		TransmitActionPerformed();
		counter_ = 0;
		active_ = false;
		if(m_bShowLog)
		{
			LOG_ERROR(cString::Format("RunCrossParkingScript: Cross Parking Ended"));
		}
	}
}

tVoid cActionFilter::RunParallelParkingScript()
{
    /************ Parallel Parking Properties *******************/
	GetParallelParkingProperties();
	TransmitRightIndicator(tTrue,m_tsArduinoTime);
	if(m_bShowLog)
	{
		LOG_ERROR(cString::Format("RunParallelParkingScript: Parallel Parking Started"));
	}
	if (counter_ < pp_straightDuration )
	{
		TransmitAcceleration(pp_straightSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(pp_straightAngle,m_tsArduinoTime);
		counter_++;
	} 
	else if (counter_ < (pp_straightDuration + pp_forwardLeftDuration) )
	{
		TransmitAcceleration(pp_forwardLeftSpeed ,m_tsArduinoTime);
		TransmitSteeringAngle(pp_forwardLeftAngle,m_tsArduinoTime);
		counter_++;
	}
	else if (counter_ < (pp_straightDuration + pp_forwardLeftDuration + pp_rightDuration) )
	{
		TransmitAcceleration(pp_rightSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(pp_rightAngle,m_tsArduinoTime);
		counter_++;
	}
	else if (counter_ <(pp_straightDuration + pp_forwardLeftDuration+ pp_leftDuration + pp_rightDuration) )
	{
		TransmitReverseIndicator(tTrue,m_tsArduinoTime);
		TransmitAcceleration(pp_leftSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(pp_leftAngle,m_tsArduinoTime);
		counter_++;
	}
	else if (counter_ < (pp_straightDuration + pp_forwardLeftDuration + pp_leftDuration + pp_rightDuration + pp_backDuration))
	{
		TransmitReverseIndicator(tTrue,m_tsArduinoTime);
		TransmitAcceleration(pp_backSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(pp_backAngle,m_tsArduinoTime);
		counter_++;
	}
	else if (counter_ < (pp_straightDuration + pp_forwardLeftDuration  + pp_leftDuration + pp_rightDuration + pp_backDuration + pp_ForwardLastDuration))
	{
		TransmitReverseIndicator(tTrue,m_tsArduinoTime);
		TransmitAcceleration(pp_ForwardLastSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(pp_ForwardLastAngle,m_tsArduinoTime);
		counter_++;
	}
	else 
	{
		TransmitAcceleration(0.0f,m_tsArduinoTime);
		TransmitSteeringAngle(pp_straightAngle,m_tsArduinoTime);
		TransmitRightIndicator(tFalse,m_tsArduinoTime);
		TransmitReverseIndicator(tFalse,m_tsArduinoTime);
		TransmitActionPerformed();
		counter_ = 0;
		active_ = false;
		if(m_bShowLog)
		{
			LOG_ERROR(cString::Format("RunParallelParkingScript: Parallel Parking Ended"));
		}
	}
}

tVoid cActionFilter::RunPullOverRightScript()
{
    /************ PullOver Right Properties *******************/
	GetPullOverRightProperties();
	TransmitRightIndicator(tTrue,m_tsArduinoTime);
	if(m_bShowLog)
	{
		LOG_ERROR(cString::Format("RunPullOverRightScript: Started"));
	}
	if (counter_ < pr_straightDuration )
	{
		TransmitAcceleration(pr_straightSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(pr_straightAngle,m_tsArduinoTime);
		counter_++;
	} 
	else if (counter_ < (pr_straightDuration + pr_rightDuration) )
	{
		TransmitAcceleration(pr_rightSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(pr_rightAngle,m_tsArduinoTime);
		counter_++;
	}
	else 
	{
		TransmitAcceleration(0.0f,m_tsArduinoTime);
		TransmitSteeringAngle(90.0,m_tsArduinoTime);
		TransmitRightIndicator(tFalse,m_tsArduinoTime);
		TransmitActionPerformed();
		counter_ = 0;
		active_ = false;
		cp_counter = 0;
			pp_counter = 0;
		if(m_bShowLog)
		{
			LOG_ERROR(cString::Format("RunPullOverRightScript: Ended"));
		}
	}
	
}

tVoid cActionFilter::RunPullOverLeftScript()
{
    /************ PullOver Left Properties *******************/
	GetPullOverLeftProperties();
	TransmitLeftIndicator(tTrue,m_tsArduinoTime);
	if(m_bShowLog)
	{
		LOG_ERROR(cString::Format("RunPullOverLeftScript: Started"));
	}
	if (counter_ < pl_straightDuration )
	{
		TransmitAcceleration(pl_straightSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(pl_straightAngle,m_tsArduinoTime);
		counter_++;
	} 
	else if (counter_ < (pl_straightDuration + pl_leftDuration) )
	{
		TransmitAcceleration(pl_leftSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(pl_leftAngle,m_tsArduinoTime);
		counter_++;
	}
	else 
	{
		TransmitAcceleration(0.0f,m_tsArduinoTime);
		TransmitSteeringAngle(90.0,m_tsArduinoTime);
		TransmitLeftIndicator(tFalse,m_tsArduinoTime);
		TransmitActionPerformed();
		counter_ = 0;
		active_ = false;
		cp_counter = 0;
			pp_counter = 0;
		if(m_bShowLog)
		{
			LOG_ERROR(cString::Format("RunPullOverLeftScript: Ended"));
		}
	}
}

tVoid cActionFilter::RunPullOverRightFromParallelParkingScript()
{
		/************ PullOver Left Properties *******************/
	GetPullOverRightFromParallelParkingProperties();
	TransmitRightIndicator(tTrue,m_tsArduinoTime);
	if(m_bShowLog)
	{
		LOG_ERROR(cString::Format("RunPullOverRightFromParallelParkingScript: Started"));
	}
	if (counter_ < pr_pp_backDuration )
	{
		TransmitAcceleration(pr_pp_backSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(pr_pp_backAngle,m_tsArduinoTime);
		counter_++;
	} 
	else if (counter_ < (pr_pp_backDuration + pr_pp_leftDuration) )
	{
		TransmitAcceleration(pr_pp_leftSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(pr_pp_leftAngle,m_tsArduinoTime);
		counter_++;
	}
	else if (counter_ < (pr_pp_backDuration + pr_pp_leftDuration + pr_pp_rightDuration) )
	{
		TransmitAcceleration(pr_pp_rightSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(pr_pp_rightAngle,m_tsArduinoTime);
		counter_++;
	}
	else 
	{
		TransmitAcceleration(0.0f,m_tsArduinoTime);
		TransmitSteeringAngle(90.0,m_tsArduinoTime);
		TransmitRightIndicator(tFalse,m_tsArduinoTime);
		TransmitActionPerformed();
		counter_ = 0;
		active_ = false;
		cp_counter = 0;
		pp_counter = 0;
		if(m_bShowLog)
		{
			LOG_ERROR(cString::Format("RunPullOverRightFromParallelParkingScript: Ended"));
		}
	}
}

tVoid cActionFilter::RunPullOverLeftFromParallelParkingScript()
{
	/************ PullOver Right Properties *******************/
	GetPullOverLeftFromParallelParkingProperties();
	TransmitLeftIndicator(tTrue,m_tsArduinoTime);
	if(m_bShowLog)
	{
		LOG_ERROR(cString::Format("RunPullOverLeftFromParallelParkingScript: Started"));
	}
	if (counter_ < pl_pp_backDuration )
	{
		TransmitAcceleration(pl_pp_backSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(pl_pp_backAngle,m_tsArduinoTime);
		counter_++;
	} 
	else if (counter_ < (pl_pp_backDuration + pl_pp_forwardDuration) )
	{
		TransmitAcceleration(pl_pp_forwardSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(pl_pp_forwardAngle,m_tsArduinoTime);
		counter_++;
	}
	else if (counter_ < (pl_pp_backDuration + pl_pp_leftDuration + pl_pp_forwardDuration) )
	{
		TransmitAcceleration(pl_pp_leftSpeed,m_tsArduinoTime);
		TransmitSteeringAngle(pl_pp_leftAngle,m_tsArduinoTime);
		counter_++;
	}
	else 
	{
		TransmitAcceleration(0.0f,m_tsArduinoTime);
		TransmitSteeringAngle(90.0,m_tsArduinoTime);
		TransmitLeftIndicator(tFalse,m_tsArduinoTime);
		TransmitActionPerformed();
		counter_ = 0;
		active_ = false;
		cp_counter = 0;
		pp_counter = 0;
		if(m_bShowLog)
		{
			LOG_ERROR(cString::Format("RunPullOverLeftFromParallelParkingScript: Ended"));
		}
	}
}

/****************** Properties Setter Methods **********************/
tVoid cActionFilter::SetParallelParkingProperties()
{
	/************* Parallel Parking Properties *******************/
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_ANGLE,90.0);
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_RIGHT_ANGLE,120.0);
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_LEFT_ANGLE,60.0);
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_BACKWARD_ANGLE,90.0);
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_LEFT_ANGLE,60.0);
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_LAST_ANGLE,120.0);
	
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_LEFT_SPEED,0.6);
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_SPEED,1.0);
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_RIGHT_SPEED,-0.6);
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_LEFT_SPEED,-0.6);
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_BACKWARD_SPEED,-0.6);
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_LAST_SPEED,-0.6);

	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_FORWARD,50);
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_LEFT_TURNING,30);
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_RIGHT_TURNING,40);
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_BACKWARD,20);
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_FORWARD_LEFT,0);
	SetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_FORWARD_LAST,15);
}

tVoid cActionFilter::SetCrossParkingProperties()
{
	/************* Cross Parking Properties *******************/
	SetPropertyFloat(AF_PROP_CROSS_PARKING_LEFT_ANGLE,60.0);
	SetPropertyFloat(AF_PROP_CROSS_PARKING_STRAIGHT_ANGLE,90.0);
	SetPropertyFloat(AF_PROP_CROSS_PARKING_RIGHT_ANGLE,120.0);
	SetPropertyFloat(AF_PROP_CROSS_PARKING_BACK_ANGLE,90.0);

	SetPropertyFloat(AF_PROP_CROSS_PARKING_LEFT_SPEED,1.0);
	SetPropertyFloat(AF_PROP_CROSS_PARKING_STRAIGHT_SPEED,1.0);
	SetPropertyFloat(AF_PROP_CROSS_PARKING_RIGHT_SPEED,-1.0);
	SetPropertyFloat(AF_PROP_CROSS_PARKING_BACKWARD_SPEED,-1.0);


	SetPropertyFloat(AF_PROP_CROSS_PARKING_DURATION_FORWARD,35);
	SetPropertyFloat(AF_PROP_CROSS_PARKING_DURATION_LEFT_TURNING,40);
	SetPropertyFloat(AF_PROP_CROSS_PARKING_DURATION_RIGHT_TURNING,45);
	SetPropertyFloat(AF_PROP_CROSS_PARKING_DURATION_BACKWARD,35);

}

tVoid cActionFilter::SetStraightProperties()
{
	/************* Straight Properties *****************/
	SetPropertyFloat(AF_PROP_STRAIGHT_SPEED,1.0);
	SetPropertyFloat(AF_PROP_STRAIGHT_DURATION_FORWARD,50.0);
	SetPropertyFloat(AF_PROP_STRAIGHT_STRAIGHT_ANGLE,90.0);
}

tVoid cActionFilter::SetLeftTurnProperties()
{
	/************* Turn Left Properties ****************/
	SetPropertyFloat(AF_PROP_LEFT_ANGLE,50.0);
	SetPropertyFloat(AF_PROP_LEFT_DURATION_FORWARD,20.0);
	SetPropertyFloat(AF_PROP_LEFT_DURATION_TURNING,80.0);
	SetPropertyFloat(AF_PROP_LEFT_SPEED,1.0);
	SetPropertyFloat(AF_PROP_LEFT_STRAIGHT_ANGLE,90.0);
	SetPropertyFloat(AF_PROP_LEFT_STRAIGHT_SPEED,1.0);

	SetPropertyBool(AF_PROP_LEFT_ANGLE NSSUBPROP_ISCHANGEABLE,tTrue);
	SetPropertyStr(AF_PROP_LEFT_ANGLE NSSUBPROP_DESCRIPTION, "Angle for left turn");
}

tVoid cActionFilter::SetRightTurnProperties()
{
	/************* Turn Right Properties *******************/
	SetPropertyFloat(AF_PROP_RIGHT_ANGLE,50.0);
	SetPropertyFloat(AF_PROP_RIGHT_DURATION_FORWARD,20.0);
	SetPropertyFloat(AF_PROP_RIGHT_DURATION_TURNING,80.0);
	SetPropertyFloat(AF_PROP_RIGHT_SPEED,1.0);

	SetPropertyFloat(AF_PROP_RIGHT_STRAIGHT_ANGLE,90.0);
	SetPropertyFloat(AF_PROP_RIGHT_STRAIGHT_SPEED,1.0);
	SetPropertyFloat(AF_PROP_RIGHT_STRAIGHT_DISTANCE,0.0);

	SetPropertyFloat(AF_PROP_RIGHT_DISTANCE,10);
	SetPropertyFloat(AF_PROP_RIGHT_TOTAL_DISTANCE,10);


}

tVoid cActionFilter::SetPullOverLeftProperties()
{
	/************* Pull Over left Properties *******************/
	SetPropertyFloat(AF_PROP_PL_LEFT_SPEED,1.0);
	SetPropertyFloat(AF_PROP_PL_LEFT_ANGLE,60.0);
	SetPropertyFloat(AF_PROP_PL_LEFT_DURATION,80.0);

	SetPropertyFloat(AF_PROP_PL_LEFT_STRAIGHT_SPEED,1.0);
	SetPropertyFloat(AF_PROP_PL_LEFT_STRAIGHT_ANGLE,90.0);
	SetPropertyFloat(AF_PROP_PL_LEFT_DURATION_STRAIGHT,20.0);
}

tVoid cActionFilter::SetPullOverRightProperties()
{
	/************* Pull Over Right Properties *******************/
	SetPropertyFloat(AF_PROP_PR_RIGHT_SPEED,1.0);
	SetPropertyFloat(AF_PROP_PR_RIGHT_ANGLE,60.0);
	SetPropertyFloat(AF_PROP_PR_RIGHT_DURATION,80.0);

	SetPropertyFloat(AF_PROP_PR_RIGHT_STRAIGHT_SPEED,1.0);
	SetPropertyFloat(AF_PROP_PR_RIGHT_STRAIGHT_ANGLE,90.0);
	SetPropertyFloat(AF_PROP_PR_RIGHT_DURATION_STRAIGHT,20.0);
}

tVoid cActionFilter::SetPullOverRightFromParallelParkingProperties()
{
	/************* Pull Over Parallel Right Properties *******************/
	SetPropertyFloat(AF_PROP_PR_PP_BACK_SPEED,-1.0);
	SetPropertyFloat(AF_PROP_PR_PP_LEFT_SPEED,1.0);
	SetPropertyFloat(AF_PROP_PR_PP_RIGHT_SPEED,1.0);

	SetPropertyFloat(AF_PROP_PR_PP_BACK_ANGLE,90.0);
	SetPropertyFloat(AF_PROP_PR_PP_LEFT_ANGLE,120.0);
	SetPropertyFloat(AF_PROP_PR_PP_RIGHT_ANGLE,60.0);

	SetPropertyFloat(AF_PROP_PR_PP_BACK_DURATION,10);
	SetPropertyFloat(AF_PROP_PR_PP_LEFT_DURATION,20);
	SetPropertyFloat(AF_PROP_PR_PP_RIGHT_DURATION,30);
}

tVoid cActionFilter::SetPullOverLeftFromParallelParkingProperties()
{
		/************* Pull Over Parallel left Properties *******************/
	SetPropertyFloat(AF_PROP_PL_PP_BACK_SPEED,-1.0);
	SetPropertyFloat(AF_PROP_PL_PP_FORWARD_SPEED,1.0);
	SetPropertyFloat(AF_PROP_PL_PP_LEFT_SPEED,1.0);

	SetPropertyFloat(AF_PROP_PL_PP_BACK_ANGLE,60.0);
	SetPropertyFloat(AF_PROP_PL_PP_FORWARD_ANGLE,90.0);
	SetPropertyFloat(AF_PROP_PL_PP_LEFT_ANGLE,120.0);

	SetPropertyFloat(AF_PROP_PL_PP_BACK_DURATION,10);
	SetPropertyFloat(AF_PROP_PL_PP_FORWARD_DURATION,10);	
	SetPropertyFloat(AF_PROP_PL_PP_LEFT_DURATION,50);
	
}

/****************** Properties Getter Methods **********************/
tVoid cActionFilter::GetStraightProperties()
{
    /************ Straight Properties ****************/
	st_straightAngle = GetPropertyFloat(AF_PROP_STRAIGHT_STRAIGHT_ANGLE);
	st_straightDuration = GetPropertyFloat(AF_PROP_STRAIGHT_DURATION_FORWARD);
	st_straightSpeed = GetPropertyFloat(AF_PROP_STRAIGHT_SPEED);
}

tVoid cActionFilter::GetLeftTurnProperties()
{
    /************ Left Turn Properties ****************/
	lt_leftAngle = GetPropertyFloat(AF_PROP_LEFT_ANGLE);
	lt_leftDuration = GetPropertyFloat(AF_PROP_LEFT_DURATION_TURNING);
	lt_leftSpeed = GetPropertyFloat(AF_PROP_LEFT_SPEED);

	lt_straightAngle = GetPropertyFloat(AF_PROP_LEFT_STRAIGHT_ANGLE);
	lt_straightDuration = GetPropertyFloat(AF_PROP_LEFT_DURATION_FORWARD);
	lt_straightSpeed = GetPropertyFloat(AF_PROP_LEFT_STRAIGHT_SPEED);
	
}

tVoid cActionFilter::GetRightTurnProperties()
{
    /************ Right Turn Properties ****************/
	rt_rightAngle = GetPropertyFloat(AF_PROP_RIGHT_ANGLE);
	rt_rightDuration = GetPropertyFloat(AF_PROP_RIGHT_DURATION_TURNING);
	rt_rightSpeed = GetPropertyFloat(AF_PROP_RIGHT_SPEED);

	rt_straightAngle = GetPropertyFloat(AF_PROP_RIGHT_STRAIGHT_ANGLE);
	rt_straightDuration = GetPropertyFloat(AF_PROP_RIGHT_DURATION_FORWARD);
	rt_straightSpeed = GetPropertyFloat(AF_PROP_RIGHT_STRAIGHT_SPEED);
}

tVoid cActionFilter::GetCrossParkingProperties()
{
    /************ Cross Parking Properties ****************/
	cp_leftAngle =  GetPropertyFloat(AF_PROP_CROSS_PARKING_LEFT_ANGLE);
	cp_straightAngle =  GetPropertyFloat(AF_PROP_CROSS_PARKING_STRAIGHT_ANGLE);
	cp_rightAngle =  GetPropertyFloat(AF_PROP_CROSS_PARKING_RIGHT_ANGLE);
	cp_backAngle  =  GetPropertyFloat(AF_PROP_CROSS_PARKING_BACK_ANGLE);

	cp_leftSpeed = GetPropertyFloat(AF_PROP_CROSS_PARKING_LEFT_SPEED);
	cp_straightSpeed = GetPropertyFloat(AF_PROP_CROSS_PARKING_STRAIGHT_SPEED);
	cp_rightSpeed = GetPropertyFloat(AF_PROP_CROSS_PARKING_RIGHT_SPEED);
	cp_backSpeed = GetPropertyFloat(AF_PROP_CROSS_PARKING_BACKWARD_SPEED);

	cp_leftDuration = GetPropertyFloat(AF_PROP_CROSS_PARKING_DURATION_LEFT_TURNING);
	cp_straightDuration = GetPropertyFloat(AF_PROP_CROSS_PARKING_DURATION_FORWARD);
	cp_rightDuration = GetPropertyFloat(AF_PROP_CROSS_PARKING_DURATION_RIGHT_TURNING);
	cp_backDuration = GetPropertyFloat(AF_PROP_CROSS_PARKING_DURATION_BACKWARD);
}

tVoid cActionFilter::GetParallelParkingProperties()
{
	/************ Parallel Parking Properties ****************/
	pp_leftAngle =  GetPropertyFloat(AF_PROP_PARALLEL_PARKING_LEFT_ANGLE);
	pp_straightAngle =  GetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_ANGLE);
	pp_rightAngle =  GetPropertyFloat(AF_PROP_PARALLEL_PARKING_RIGHT_ANGLE);
	pp_backAngle  =  GetPropertyFloat(AF_PROP_PARALLEL_PARKING_BACKWARD_ANGLE);
	pp_ForwardLastAngle = GetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_FORWARD_LAST);

	pp_leftSpeed = GetPropertyFloat(AF_PROP_PARALLEL_PARKING_LEFT_SPEED);
	pp_straightSpeed = GetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_SPEED);
	pp_rightSpeed = GetPropertyFloat(AF_PROP_PARALLEL_PARKING_RIGHT_SPEED);
	pp_backSpeed = GetPropertyFloat(AF_PROP_PARALLEL_PARKING_BACKWARD_SPEED);
	pp_ForwardLastSpeed = GetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_LAST_SPEED);
	

	pp_leftDuration = GetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_LEFT_TURNING);
	pp_straightDuration = GetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_FORWARD);
	pp_rightDuration = GetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_RIGHT_TURNING);
	pp_backDuration = GetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_BACKWARD);
	pp_ForwardLastDuration = GetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_FORWARD_LAST);

	pp_forwardLeftAngle = GetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_LEFT_ANGLE);
	pp_forwardLeftSpeed = GetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_LEFT_SPEED);
	pp_forwardLeftDuration = GetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_FORWARD_LEFT);
}

tVoid cActionFilter::GetPullOverLeftProperties()
{
	/************* Pull Over Left Properties *******************/
	pl_leftAngle = GetPropertyFloat(AF_PROP_PL_LEFT_ANGLE);
	pl_leftSpeed = GetPropertyFloat(AF_PROP_PL_LEFT_SPEED);
	pl_leftDuration = GetPropertyFloat(AF_PROP_PL_LEFT_DURATION);

	pl_straightAngle = GetPropertyFloat(AF_PROP_PL_LEFT_STRAIGHT_ANGLE);
	pl_straightSpeed = GetPropertyFloat(AF_PROP_PL_LEFT_STRAIGHT_SPEED);
	pl_straightDuration = GetPropertyFloat(AF_PROP_PL_LEFT_DURATION_STRAIGHT);
	
}

tVoid cActionFilter::GetPullOverRightProperties()
{
	/************* Pull Over Right Properties *******************/
	pr_rightAngle = GetPropertyFloat(AF_PROP_PR_RIGHT_ANGLE);
	pr_rightSpeed = GetPropertyFloat(AF_PROP_PR_RIGHT_SPEED);
	pr_rightDuration = GetPropertyFloat(AF_PROP_PR_RIGHT_DURATION);

	pr_straightAngle = GetPropertyFloat(AF_PROP_PR_RIGHT_STRAIGHT_ANGLE);
	pr_straightSpeed = GetPropertyFloat(AF_PROP_PR_RIGHT_STRAIGHT_SPEED);
	pr_straightDuration = GetPropertyFloat(AF_PROP_PR_RIGHT_DURATION_STRAIGHT);
}

tVoid cActionFilter::GetPullOverRightFromParallelParkingProperties()
{
	/************* Pull Over Parallel left Properties *******************/
	pr_pp_backSpeed = GetPropertyFloat(AF_PROP_PR_PP_BACK_SPEED);
	pr_pp_leftSpeed = GetPropertyFloat(AF_PROP_PR_PP_LEFT_SPEED);
	pr_pp_rightSpeed = GetPropertyFloat(AF_PROP_PR_PP_RIGHT_SPEED);

	pr_pp_backAngle = GetPropertyFloat(AF_PROP_PR_PP_BACK_ANGLE);
	pr_pp_leftAngle = GetPropertyFloat(AF_PROP_PR_PP_LEFT_ANGLE);
	pr_pp_rightAngle = GetPropertyFloat(AF_PROP_PR_PP_RIGHT_ANGLE);

	pr_pp_backDuration = GetPropertyFloat(AF_PROP_PR_PP_BACK_DURATION);
	pr_pp_leftDuration = GetPropertyFloat(AF_PROP_PR_PP_LEFT_DURATION);
	pr_pp_rightDuration = GetPropertyFloat(AF_PROP_PR_PP_RIGHT_DURATION);
}

tVoid cActionFilter::GetPullOverLeftFromParallelParkingProperties()
{
	/************* Pull Over Right Properties *******************/
	pl_pp_backSpeed = GetPropertyFloat(AF_PROP_PL_PP_BACK_SPEED);
	pl_pp_leftSpeed = GetPropertyFloat(AF_PROP_PL_PP_LEFT_SPEED);
	pl_pp_forwardSpeed = GetPropertyFloat(AF_PROP_PL_PP_FORWARD_SPEED);

	pl_pp_backAngle = GetPropertyFloat(AF_PROP_PL_PP_BACK_ANGLE);
	pl_pp_leftAngle = GetPropertyFloat(AF_PROP_PL_PP_LEFT_ANGLE);
	pl_pp_forwardAngle = GetPropertyFloat(AF_PROP_PL_PP_FORWARD_ANGLE);

	pl_pp_backDuration = GetPropertyFloat(AF_PROP_PL_PP_BACK_DURATION);
	pl_pp_leftDuration = GetPropertyFloat(AF_PROP_PL_PP_LEFT_DURATION);
	pl_pp_forwardDuration = GetPropertyFloat(AF_PROP_PL_PP_FORWARD_DURATION);
}
