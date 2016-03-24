#include "stdafx.h"
#include "daedalusDecision.h"
#include "aadc_roadSign_enums.h"
#include "daedalus_action_enums.h"
#include "daedalus_action_constant.h"
#include "../utile/object/object.h"
#include "constant.h"
#include "IPM.h"
#include "ParkingType.h"

// Create filter shell
ADTF_FILTER_PLUGIN("DAEDALUS Decision Filter", OID_ADTF_DAEDALUS_DECISION_FILTER, cDaedalusDecision)

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

cDaedalusDecision::cDaedalusDecision(const tChar* __info):cFilter(__info){
    SetPropertyInt("CountObjectDetection", 3);
    SetPropertyInt("CountConfirmDetection", 5);
    SetPropertyInt("SlowSpeedForMarker", 10);
    SetPropertyInt("slowIterationsAfterTurn", 40);
    SetPropertyFloat("MarkerAreaThreshhold", 550.0);
    SetPropertyFloat("DistanceCrossroadToMarker", 1.90 );
    SetPropertyFloat("ExtraDistance",0.0);
    SetPropertyFloat("MarkerRotationThreshhold", 1.80);
    SetPropertyFloat("DistanceErrorThreshhold", 0.05);
    SetPropertyInt("MarkerTime_Threshold", 1000000);
    SetPropertyBool("m_bDebugModeEnabled", true);  
    SetPropertyFloat("Speed::SpeedDed_Threshold1", 0.67);
    SetPropertyFloat("Speed::SpeedDed_Threshold2", 0.50);
    SetPropertyFloat("Speed::SpeedDed_Threshold3", 0.60);
    SetPropertyFloat("Speed::SpeedDed_Threshold4", 0.80);
    SetPropertyFloat("Speed::SpeedDed_Threshold5", 0.80);
    SetPropertyFloat("Speed::SpeedDed_Threshold6", 0.80);
    SetPropertyFloat("Speed::SpeedDed_Threshold7", 1.2);
    SetPropertyFloat("RightTurnExtraDistance", -0.1);
    SetPropertyInt("StoppingIterations",100);
    SetPropertyInt("ParkingTypeDectectionIteration",19);
    SetPropertyInt("BlinkingInParkingIter", 100);
    SetPropertyInt("waitBeforeChangeLane",100);
    SetPropertyInt("ObsCR::MaxWidth",320);
    SetPropertyInt("ObsCR::MinWidth",40);
    SetPropertyInt("ObsCR::MinHeight",70);
    SetPropertyInt("ObsCR::MaxHeight",200);
    SetPropertyInt("ObsLane::LeftOffset",90);
    SetPropertyInt("ObsLane::RightOffset",270);
    SetPropertyInt("ObsLane::MinWidth",120);
    SetPropertyInt("ObsLane::MaxWidth",280);
    SetPropertyInt("ObsReturn::RightOffset",250);
    SetPropertyFloat("ObsReturn::FrontRightThresh",0.2);
    SetPropertyFloat("ObsReturn::RightThresh",0.2);
    SetPropertyFloat("LaneReturnIterations",150);
    SetAllProperties();
}

cDaedalusDecision::~cDaedalusDecision(){}

tResult cDaedalusDecision::CreateInputPins(__exception)
{
    
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
   // Video Input -- added because of action filter merging
    RETURN_IF_FAILED(m_iVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_iVideoInputPin));
    
    //Lane Detection
     //Acceleration Input    
    tChar const * strDescLaneTrackingValue = pDescManager->GetMediaDescription("tLaneTrackingInfo");
    RETURN_IF_POINTER_NULL(strDescLaneTrackingValue);
    cObjectPtr<IMediaType> tLaneDetectionInfo = new cMediaType(0, 0, 0, "tLaneTrackingInfo", strDescLaneTrackingValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(tLaneDetectionInfo->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputLane));

    RETURN_IF_FAILED(m_iLaneTrackingStructInput.Create("LaneTrackingInfo", tLaneDetectionInfo  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_iLaneTrackingStructInput));
    //Marker 
    // create the description for the road sign pin
    tChar const * strDescExt = pDescManager->GetMediaDescription("tRoadSignExt");   
    RETURN_IF_POINTER_NULL(strDescExt);    
    cObjectPtr<IMediaType> pTypeExt = new cMediaType(0, 0, 0, "tRoadSignExt", strDescExt, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    
    // create the extended road sign OutputPin
    RETURN_IF_FAILED(m_iPinRoadSignExt.Create("RoadSign_ext", pTypeExt, this));
    RETURN_IF_FAILED(RegisterPin(&m_iPinRoadSignExt));
    // set the description for the extended road sign pin
    RETURN_IF_FAILED(pTypeExt->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputMarker));
    
    // Driver Module
    tChar const * strDescIdValue = pDescManager->GetMediaDescription("tIdValue");    
    RETURN_IF_POINTER_NULL(strDescIdValue);
    cObjectPtr<IMediaType> pTypeIdValue = new cMediaType(0, 0, 0, "tIdValue", strDescIdValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    
    RETURN_IF_FAILED(pTypeIdValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionDecisionInput)); 
    RETURN_IF_FAILED(m_iDriverAction.Create("Action",pTypeIdValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_iDriverAction));

    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);        
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSampleDistance));
    RETURN_IF_FAILED(m_iWheelEncoderDistance.Create("DistanceSample", pTypeSignalValue, static_cast<IPinEventSink*> (this)));   
    RETURN_IF_FAILED(RegisterPin(&m_iWheelEncoderDistance));
    
    // Object pin
    tChar const * object_stream_type = pDescManager->GetMediaDescription("ObjectArray");
    RETURN_IF_POINTER_NULL(object_stream_type);
    cObjectPtr<IMediaType> object_type_signal_value = new cMediaType(0, 0, 0, "ObjectArray", object_stream_type, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(object_type_signal_value->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION,(tVoid**) &m_pDescriptionOutputObject));
    RETURN_IF_FAILED(m_iObjectFilter.Create("objects", object_type_signal_value, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_iObjectFilter));
    
    tChar const * output_stream_type2 = pDescManager->GetMediaDescription("tObstacleDistanceStruct");
    RETURN_IF_POINTER_NULL(output_stream_type2);
    cObjectPtr<IMediaType> output_type_signal_value2 = new cMediaType(0, 0, 0, "tObstacleDistanceStruct", output_stream_type2,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(output_type_signal_value2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSensor));
  
    RETURN_IF_FAILED(m_iSensorFilter.Create("Emergency_Sensor", output_type_signal_value2, static_cast<IPinEventSink*> (this)));    
    RETURN_IF_FAILED(RegisterPin(&m_iSensorFilter));
    
    
    tChar const * strDescSignalValue2 = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue2);        
    cObjectPtr<IMediaType> pTypeSignalValue2 = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue2,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescParkingDetectionFilter));
    RETURN_IF_FAILED(m_oParkingDetectionInput.Create("Parking_Detection", pTypeSignalValue2, static_cast<IPinEventSink*> (this)));   
    RETURN_IF_FAILED(RegisterPin(&m_oParkingDetectionInput));


    tChar const * CrDetDescSignalValue2 = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(CrDetDescSignalValue2);        
    cObjectPtr<IMediaType> pTypeSignalValue3 = new cMediaType(0, 0, 0, "tSignalValue", CrDetDescSignalValue2,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue3->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescCrossroadDetectionFilter));
    RETURN_IF_FAILED(m_oCrossroadDetectionInput.Create("Crossroad_Detection", pTypeSignalValue3, static_cast<IPinEventSink*> (this)));   
    RETURN_IF_FAILED(RegisterPin(&m_oCrossroadDetectionInput));
    
    RETURN_NOERROR;
}


tResult cDaedalusDecision::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // Media Description Signal
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);        
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal)); 
    
    // Media Description Bool
    tChar const * strDescSignalBoolValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalBoolValue);
    cObjectPtr<IMediaType> pTypeSignalBoolValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalBoolValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(pTypeSignalBoolValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescBool));
    
    //Acceleration Output
    RETURN_IF_FAILED(m_oAccelerateOutputPin.Create("SpeedController", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oAccelerateOutputPin));
    
    // Steer Angle Output pin
    RETURN_IF_FAILED(m_oSteerOutputPin.Create("SteeringController", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oSteerOutputPin));

    // Beam Output pin
    RETURN_IF_FAILED(m_oHeadlightOutputPin.Create("headLightEnabled", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oHeadlightOutputPin));
    
    // Beam Output pin
    RETURN_IF_FAILED(EnableLaneTrackingPin.Create("StartLaneTracking", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&EnableLaneTrackingPin));
    
    // Indicator Output pin
    RETURN_IF_FAILED(m_oTurnSignalLeftOutputPin.Create("turnSignalLeftEnabled", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oTurnSignalLeftOutputPin));

    RETURN_IF_FAILED(m_oTurnSignalRightOutputPin.Create("turnSignalRightEnabled", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oTurnSignalRightOutputPin));

     // hazzard Output pin
    RETURN_IF_FAILED(m_oHazardIndicatorPin.Create("hazzardLightsEnabled", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oHazardIndicatorPin));
 
    // Brakelight Output pin
    RETURN_IF_FAILED(m_oBrakeLightOutputPin.Create("brakeLightEnabled", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oBrakeLightOutputPin));

    // Reverse Output pin
    RETURN_IF_FAILED(m_oReverseIndicatorOutputPin.Create("ReverseLightEnabled", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oReverseIndicatorOutputPin));
    
    // Feedback to DriverModule
    tChar const * strDescIdValue = pDescManager->GetMediaDescription("tIdValue");    
    RETURN_IF_POINTER_NULL(strDescIdValue);
    cObjectPtr<IMediaType> pTypeIdValue = new cMediaType(0, 0, 0, "tIdValue", strDescIdValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    
    RETURN_IF_FAILED(pTypeIdValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionDecisionOutput));    
    RETURN_IF_FAILED(m_oDriverModuleFeedback.Create("FeedbackToDriverFilter",pTypeIdValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oDriverModuleFeedback));
    // create new output pins
    
    tChar const * strDescSignalValue2 = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue2);        
    cObjectPtr<IMediaType> pTypeSignalValue2 = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalValue2,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescEnableParkingDetectionFilter));
    RETURN_IF_FAILED(m_oEnableParkingDetectionFilter.Create("EnableParkingDistance", pTypeSignalValue2, static_cast<IPinEventSink*> (this)));   
    RETURN_IF_FAILED(RegisterPin(&m_oEnableParkingDetectionFilter));


    tChar const * strDescSignalValue3 = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue3);        
    cObjectPtr<IMediaType> pTypeSignalValue3 = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalValue3,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue3->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescCrossroadDetectionInputFilter));
    RETURN_IF_FAILED(m_oEnableCrossroadDetectionFilter.Create("CrossRoadDetection", pTypeSignalValue3, static_cast<IPinEventSink*> (this)));   
    RETURN_IF_FAILED(RegisterPin(&m_oEnableCrossroadDetectionFilter));
    
    RETURN_NOERROR;
}

tResult cDaedalusDecision::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    
    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);
    }
    else if (eStage == StageNormal)
    {
        // Action Filter properties
        CrossRoadDetected = 0;
        Count_BackToLane = 0;
        us_front = 0.0f;
        us_front_left = 0.0f;
        us_front_right = 0.0f;
        us_right = 0.0f;
        waitBeforeChangeLane=0;
        CurrLane = 0;
        parkedBlinking=0;
        execFlow = 1;
        EncoderState = 0;
        WheelEncoderDistance = 0.0f;
        markerInSight = 0;
        DriveSlowAfterTurn = 0;
        startDriving = false;
        ObjectsDetectedTime = 0;
        m_bDebugModeEnabled = GetPropertyBool("m_bDebugModeEnabled");
        //input helper flag
        m_bIDsRoadSignSet = tFalse;
        LaneDetectionBufferReaderFlag = tFalse;
        DriverModuleBufferReaderFlag = tFalse;
        WheelEncoderModuleBufferReaderFlag = tFalse;
        ObstacleSensorModuleBufferReaderFlag = tFalse;
        ParkingDetectionModuleBufferReaderFlag = tFalse;
        CrossroadDetectionModuleBufferReaderFlag = tFalse;
        //initialize flags
        Flag_UNMARKEDINTERSECTION=tFalse;
        Flag_STOPANDGIVEWAY=tFalse;
        Flag_PARKINGAREA=tFalse;
        Flag_ParkingDetection=false;
        Flag_HAVEWAY=tFalse;
        Flag_AHEADONLY=tFalse;
        Flag_GIVEWAY=tFalse;
        Flag_PEDESTRIANCROSSING=tFalse;
        Flag_NOOVERTAKING=tFalse;
        // Flag_ROUNDABOUT=tFalse;
        // Flag_NOENTRYVEHICULARTRAFFIC=tFalse;
        // Flag_ONEWAYSTREET=tFalse;
        Flag_NOMATCH=tTrue;
        Count_UNMARKEDINTERSECTION = 0;
        Count_STOPANDGIVEWAY=0;
        Count_PARKINGAREA=0;
        Count_HAVEWAY=0; 
        Count_AHEADONLY=0;
        Count_GIVEWAY=0;
        Count_PEDESTRIANCROSSING=0;
        Count_NOOVERTAKING=0;
        CountMoveAroundObs = 0;
        LastDetTime_MoveAroundObs = 0;
        //Last Detected time variable for all markers
        LastDetTime_UNMARKEDINTERSECTION = 0;
        LastDetTime_STOPANDGIVEWAY=0;
        LastDetTime_PARKINGAREA=0;
        LastDetTime_HAVEWAY=0;
        LastDetTime_AHEADONLY=0;
        LastDetTime_GIVEWAY=0;
        LastDetTime_PEDESTRIANCROSSING=0;
        LastDetTime_NOOVERTAKING=0;
        // LastDetTime_ROUNDABOUT=0;
        // LastDetTime_NOENTRYVEHICULARTRAFFIC=0;
        // LastDetTime_ONEWAYSTREET=0;

        LanePreviousVelocity = 0.0f;
        LanePreviousTimeStamp = 0;
        LanePreviousAnglePTI = 0.0f; 
        LastLaneAceleration = 0.0f;
        LastLaneSteer = 0.0f;
        LastLaneSteerPT1 = 0.0f;
        LastlaneHeadlight = tFalse;
        FlagObs = false;
        DistanceToFollow = 0.0f;
        DistanceTraveled = 0.0f;
        DriverNextAction = -1;
        iter=0;
        InitializeProperties();
        ReadProperties();
    }
    else if (eStage == StageGraphReady)
    {
    }

    RETURN_NOERROR;
}

tResult cDaedalusDecision::Shutdown(tInitStage eStage, __exception)
{
    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cDaedalusDecision::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{

    if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        InputTimeStamp = pMediaSample->GetTime();
         //Video input -- added because of action filter merging
        if(pSource == &m_iVideoInputPin)
        {
           
            ProcessForFirstFrame();
            ProcessInput(pMediaSample, InputTimeStamp);
        }
        else if(pSource == &m_iPinRoadSignExt)
        {
            if (pMediaSample != NULL && m_pDescriptionInputMarker != NULL)
            {            
                tInt16 markerId = 0.0f;
                tFloat32 f32Area = 0.0f;
                tFloat32 TVec[] = {0.0f, 0.0f, 0.0f};
                tFloat32 RVec[] = {0.0f, 0.0f, 0.0f};
                {   // focus for sample read lock
                    // read-out the incoming Media Sample
                    __adtf_sample_read_lock_mediadescription(m_pDescriptionInputMarker,pMediaSample,pCoderInput);       
                    
                    // get IDs
                    if(!m_bIDsRoadSignSet)
                    {
                        pCoderInput->GetID("i16Identifier",m_szIDRoadSignI16Identifier);
                        pCoderInput->GetID("f32Imagesize", m_szIDRoadSignF32Imagesize);
                        pCoderInput->GetID("af32TVec[0]",m_szIDRoadSignF32Translation0);
                        pCoderInput->GetID("af32TVec[1]",m_szIDRoadSignF32Translation1);
                        pCoderInput->GetID("af32TVec[2]",m_szIDRoadSignF32Translation2);
                        pCoderInput->GetID("af32RVec[0]", m_szIDRoadSignF32Rotation0);
                        pCoderInput->GetID("af32RVec[1]", m_szIDRoadSignF32Rotation1);
                        pCoderInput->GetID("af32RVec[2]", m_szIDRoadSignF32Rotation2);
                        m_bIDsRoadSignSet = tTrue;
                    }  

                    // get the values from sample
                    pCoderInput->Get(m_szIDRoadSignI16Identifier, (tVoid*)&markerId);
                    pCoderInput->Get(m_szIDRoadSignF32Imagesize, (tVoid*)&f32Area);
                    pCoderInput->Get(m_szIDRoadSignF32Translation0, (tVoid*)&TVec[0]);
                    pCoderInput->Get(m_szIDRoadSignF32Translation1, (tVoid*)&TVec[1]);  
                    pCoderInput->Get(m_szIDRoadSignF32Translation2, (tVoid*)&TVec[2]);
                    pCoderInput->Get(m_szIDRoadSignF32Rotation0, (tVoid*)&RVec[0]);    
                    pCoderInput->Get(m_szIDRoadSignF32Rotation1, (tVoid*)&RVec[1]);    
                    pCoderInput->Get(m_szIDRoadSignF32Rotation2, (tVoid*)&RVec[2]);        
                }                 
                if(!startDriving) RETURN_NOERROR;
                // if marker area in range (>1300) and x-axis translation is positive
                RVec[1] = (RVec[1] < 0)?RVec[1]+(2*3.14):RVec[1];
                
                if(execFlow != 1)
                    RETURN_NOERROR;

                if(f32Area > GetPropertyFloat("MarkerAreaThreshhold") && f32Area < 1000.0 && TVec[0] < 0 && RVec[1] >= GetPropertyFloat("MarkerRotationThreshhold"))
                {
                    if (markerId == MARKER_ID_UNMARKEDINTERSECTION && !Flag_UNMARKEDINTERSECTION)
                    {
                        if (m_bDebugModeEnabled)   
                            LOG_ERROR(cString::Format("DecisionFilter: Marker Input: Id -> %d Area: %f, T: [%f , %f , %f], R: [%f , %f , %f]", markerId, f32Area,TVec[0],TVec[1],TVec[2], RVec[0],RVec[1],RVec[2]));
                        // cross road marker
                        int diff = InputTimeStamp - LastDetTime_UNMARKEDINTERSECTION;
                        if (diff < GetPropertyInt("MarkerTime_Threshold") || LastDetTime_UNMARKEDINTERSECTION==0)
                        {
                            markerInSight = GetPropertyInt("SlowSpeedForMarker");
                            Count_UNMARKEDINTERSECTION++;
                            LastDetTime_UNMARKEDINTERSECTION=InputTimeStamp;
                        }else
                        {
                            LastDetTime_UNMARKEDINTERSECTION = 0;
                            Count_UNMARKEDINTERSECTION = 0;
                        }
                        if (Count_UNMARKEDINTERSECTION >= GetPropertyInt("CountConfirmDetection") )
                        {
                            
                            markerInSight = 0;
                            Flag_NOMATCH=tFalse;   
                            Count_UNMARKEDINTERSECTION = 0;
                            Flag_UNMARKEDINTERSECTION=tTrue;
                            DistanceToFollow = TVec[2] - GetPropertyFloat("DistanceCrossroadToMarker");
                            if ( DriverNextAction == Driver_right )
                            {
                                DistanceToFollow += GetPropertyFloat("RightTurnExtraDistance");
                            }  
                            ChangeFlow(2);
                            TransmitDetectCrossroad(true);
                            if (m_bDebugModeEnabled)  LOG_ERROR(cString::Format("Marker Detected!! Following Lane for Dist : %f",DistanceToFollow));
                            // enable travel till crossroad Flag
                        }
                    }
                    else if (markerId == MARKER_ID_STOPANDGIVEWAY && !Flag_STOPANDGIVEWAY)
                    {
                        if (m_bDebugModeEnabled)   
                            LOG_ERROR(cString::Format("DecisionFilter: Marker Input: Id -> %d Area: %f, T: [%f , %f , %f], R: [%f , %f , %f]", markerId, f32Area,TVec[0],TVec[1],TVec[2], RVec[0],RVec[1],RVec[2]));
                        int diff = InputTimeStamp - LastDetTime_STOPANDGIVEWAY;
                        if (diff < GetPropertyInt("MarkerTime_Threshold") || LastDetTime_STOPANDGIVEWAY==0)
                        {
                            markerInSight = GetPropertyInt("SlowSpeedForMarker");
                            Count_STOPANDGIVEWAY++;
                            LastDetTime_STOPANDGIVEWAY = InputTimeStamp;
                        }else
                        {
                            LastDetTime_STOPANDGIVEWAY = 0;
                            Count_STOPANDGIVEWAY = 0;
                        }
                        
                        if (Count_STOPANDGIVEWAY >= GetPropertyInt("CountConfirmDetection") )
                        {
                            
                            markerInSight = 0;
                            Flag_NOMATCH=tFalse;   
                            Count_STOPANDGIVEWAY = 0;
                            Flag_STOPANDGIVEWAY=tTrue;
                            DistanceToFollow = TVec[2] - GetPropertyFloat("DistanceCrossroadToMarker") ;
                            if ( DriverNextAction == Driver_right )
                            {
                                DistanceToFollow += GetPropertyFloat("RightTurnExtraDistance");
                            }     
                            ChangeFlow(2);
                            TransmitDetectCrossroad(true);                       
                            if (m_bDebugModeEnabled)  LOG_ERROR(cString::Format("Marker Detected!! Following Lane for Dist : %f",DistanceToFollow));
                            // enable travel till crossroad Flag
                        }
                    }
                    else if (markerId == MARKER_ID_HAVEWAY && !Flag_HAVEWAY)
                    {
                        if (m_bDebugModeEnabled)   
                            LOG_ERROR(cString::Format("DecisionFilter: Marker Input: Id -> %d Area: %f, T: [%f , %f , %f], R: [%f , %f , %f]", markerId, f32Area,TVec[0],TVec[1],TVec[2], RVec[0],RVec[1],RVec[2]));
                        int diff = InputTimeStamp - LastDetTime_HAVEWAY;
                        if (diff < GetPropertyInt("MarkerTime_Threshold") || LastDetTime_HAVEWAY==0)
                        {
                            markerInSight = GetPropertyInt("SlowSpeedForMarker");
                            Count_HAVEWAY++;
                            LastDetTime_HAVEWAY = InputTimeStamp;
                        }else
                        {
                            LastDetTime_HAVEWAY = 0;
                            Count_HAVEWAY = 0;
                        }
                        
                        if (Count_HAVEWAY >= GetPropertyInt("CountConfirmDetection") )
                        {
                            
                            markerInSight = 0;
                            Flag_NOMATCH=tFalse;   
                            Count_HAVEWAY = 0;
                            Flag_HAVEWAY=tTrue;
                            DistanceToFollow = TVec[2] - GetPropertyFloat("DistanceCrossroadToMarker")+GetPropertyFloat("ExtraDistance");
                            if ( DriverNextAction == Driver_right )
                            {
                                DistanceToFollow += GetPropertyFloat("RightTurnExtraDistance");
                            }
                            ChangeFlow(2);
                            TransmitDetectCrossroad(true);
                            if (m_bDebugModeEnabled)  LOG_ERROR(cString::Format("Marker Detected!! Following Lane for Dist : %f",DistanceToFollow));
                            // enable travel till crossroad Flag
                        }
                    }
                    else if (markerId == MARKER_ID_GIVEWAY && !Flag_GIVEWAY)
                    {
                        if (m_bDebugModeEnabled)   
                            LOG_ERROR(cString::Format("DecisionFilter: Marker Input: Id -> %d Area: %f, T: [%f , %f , %f], R: [%f , %f , %f]", markerId, f32Area,TVec[0],TVec[1],TVec[2], RVec[0],RVec[1],RVec[2]));
                        int diff = InputTimeStamp - LastDetTime_GIVEWAY;
                        if (diff < GetPropertyInt("MarkerTime_Threshold") || LastDetTime_GIVEWAY==0)
                        {
                            markerInSight = GetPropertyInt("SlowSpeedForMarker");
                            Count_GIVEWAY++;
                            LastDetTime_GIVEWAY = InputTimeStamp;
                        }else
                        {
                            LastDetTime_GIVEWAY = 0;
                            Count_GIVEWAY = 0;
                        }
                        
                        if (Count_GIVEWAY >= GetPropertyInt("CountConfirmDetection") )
                        {
                            ChangeFlow(2);
                            TransmitDetectCrossroad(true);
                            markerInSight = 0;
                            Flag_NOMATCH=tFalse;   
                            Count_GIVEWAY = 0;
                            Flag_GIVEWAY=tTrue;
                            DistanceToFollow = TVec[2] - GetPropertyFloat("DistanceCrossroadToMarker");
                            if ( DriverNextAction == Driver_right )
                            {
                                DistanceToFollow += GetPropertyFloat("RightTurnExtraDistance");
                            } 
                            ChangeFlow(2);
                            TransmitDetectCrossroad(true);
                            if (m_bDebugModeEnabled)  LOG_ERROR(cString::Format("Marker Detected!! Following Lane for Dist : %f",DistanceToFollow));
                            // enable travel till crossroad Flag
                        }
                    }
                    else if (markerId == MARKER_ID_PEDESTRIANCROSSING && !Flag_PEDESTRIANCROSSING)
                    {
                        if (m_bDebugModeEnabled)   
                            LOG_ERROR(cString::Format("DecisionFilter: Marker Input: Id -> %d Area: %f, T: [%f , %f , %f], R: [%f , %f , %f]", markerId, f32Area,TVec[0],TVec[1],TVec[2], RVec[0],RVec[1],RVec[2]));
                        int diff = InputTimeStamp - LastDetTime_PEDESTRIANCROSSING;
                        if (diff < GetPropertyInt("MarkerTime_Threshold") || LastDetTime_PEDESTRIANCROSSING==0)
                        {
                            markerInSight = GetPropertyInt("SlowSpeedForMarker");
                            Count_PEDESTRIANCROSSING++;
                            LastDetTime_PEDESTRIANCROSSING = InputTimeStamp;
                        }else
                        {
                            LastDetTime_PEDESTRIANCROSSING = 0;
                            Count_PEDESTRIANCROSSING = 0;
                        }
                        
                        if (Count_PEDESTRIANCROSSING >= GetPropertyInt("CountConfirmDetection") )
                        {
                            // ChangeFlow(2);
                            markerInSight = 0;
                            Flag_NOMATCH=tFalse;   
                            Count_PEDESTRIANCROSSING = 0;
                            Flag_PEDESTRIANCROSSING=tTrue;
                            if (m_bDebugModeEnabled)  LOG_ERROR(cString::Format("Marker Detected!! Pedestrian Crossing"));
                        }
                    }else if (markerId == MARKER_ID_NOOVERTAKING && !Flag_NOOVERTAKING)
                    {
                        if (m_bDebugModeEnabled)   
                            LOG_ERROR(cString::Format("DecisionFilter: Marker Input: Id -> %d Area: %f, T: [%f , %f , %f], R: [%f , %f , %f]", markerId, f32Area,TVec[0],TVec[1],TVec[2], RVec[0],RVec[1],RVec[2]));
                        // cross road marker
                        int diff = InputTimeStamp - LastDetTime_NOOVERTAKING;
                        if (diff < GetPropertyInt("MarkerTime_Threshold") || LastDetTime_NOOVERTAKING==0)
                        {
                            markerInSight = GetPropertyInt("SlowSpeedForMarker");
                            Count_NOOVERTAKING++;
                            LastDetTime_NOOVERTAKING=InputTimeStamp;
                        }else
                        {
                            LastDetTime_NOOVERTAKING = 0;
                            Count_NOOVERTAKING = 0;
                        }
                        if (Count_NOOVERTAKING >= GetPropertyInt("CountObjectDetection") )
                        {
                            markerInSight = 0;
                            Flag_NOMATCH=tTrue;   
                            Count_NOOVERTAKING = 0;
                            Flag_NOOVERTAKING=tTrue;
                            if (m_bDebugModeEnabled)  LOG_ERROR(cString::Format("Marker Detected!! No Overtaking"));
                        }
                    }
                    else if (markerId == MARKER_ID_PARKINGAREA && !Flag_ParkingDetection  && !Flag_PARKINGAREA)
                    {
                        if (m_bDebugModeEnabled)   
                            LOG_ERROR(cString::Format("DecisionFilter: Marker Input: Id -> %d Area: %f, T: [%f , %f , %f], R: [%f , %f , %f]", markerId, f32Area,TVec[0],TVec[1],TVec[2], RVec[0],RVec[1],RVec[2]));
                        if(DriverNextAction != Driver_parallel_parking && DriverNextAction != Driver_cross_parking)
                            RETURN_NOERROR;
                        int diff = InputTimeStamp - LastDetTime_PARKINGAREA;
                        if (diff < GetPropertyInt("MarkerTime_Threshold") || LastDetTime_PARKINGAREA==0)
                        {
                            markerInSight = GetPropertyInt("SlowSpeedForMarker");
                            Count_PARKINGAREA++;
                            LastDetTime_PARKINGAREA = InputTimeStamp;
                        }else
                        {
                            LastDetTime_PARKINGAREA = 0;
                            Count_PARKINGAREA = 0;
                        }
                        
                        if (Count_PARKINGAREA >= GetPropertyInt("CountConfirmDetection") )
                        {
                            Flag_ParkingDetection = true;
                            TransmitDetectParking();
                            Count_PARKINGAREA = 0;
                        }
                    }
                }
                            
            }
        } 
        else if (pSource == &m_iLaneTrackingStructInput)
        {
            if (pMediaSample != NULL && m_pDescriptionInputLane != NULL)
            {
                LanePreviousVelocity = LastLaneAceleration;
                LanePreviousTimeStamp = LastLaneTimestamp;
                LanePreviousAnglePTI = LastLaneSteerPT1; 
                {   // focus for sample read lock
                    // read-out the incoming Media Sample
                    __adtf_sample_read_lock_mediadescription(m_pDescriptionInputLane,pMediaSample,pCoderInput);       
                    // get IDs
                    if(!LaneDetectionBufferReaderFlag)
                    {
                        pCoderInput->GetID("Acceleration", AccelerationBuffer);
                        pCoderInput->GetID("SteeringAngle", SteeringBuffer);
                        pCoderInput->GetID("SteeringAnglePT1", SteeringPT1Buffer);
                        pCoderInput->GetID("HeadLights", HeadlightBuffer);
                        pCoderInput->GetID("ui32ArduinoTimestamp",TimeStampBuffer);
                        LaneDetectionBufferReaderFlag = tTrue;
                    }  
                    // get the values from sample
                    pCoderInput->Get(AccelerationBuffer, (tVoid*)&LastLaneAceleration);
                    pCoderInput->Get(SteeringBuffer, (tVoid*)&LastLaneSteer);
                    pCoderInput->Get(SteeringPT1Buffer, (tVoid*)&LastLaneSteerPT1);
                    pCoderInput->Get(HeadlightBuffer, (tVoid*)&LastlaneHeadlight);
                    pCoderInput->Get(TimeStampBuffer, (tVoid*)&LastLaneTimestamp);       
                }
                //if (m_bDebugModeEnabled)   
                  //   LOG_ERROR(cString::Format("DecisionFilter: LaneTracking Data: Runs -> %d ",(!Flag_CoverDistanceBeforeTurn && !Flag_StopLaneTracking))); 
                if(startDriving && execFlow == 1) // Create var 1 == lanetracking
                {
                    MakeDecision();    
                }   
            }        
        }
        else if(pSource == &m_iDriverAction)
        {
            tUInt32 DriverActionTime = 0;
            tBool DriverIsProcessed = tFalse;
            {   // focus for sample read lock
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(m_pDescriptionDecisionInput,pMediaSample,pCoderInput);       
                // get IDs
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
            
            if(DriverNextAction == Driver_stop){
                Stop();   
            }
            else if (!startDriving){
                if(DriverNextAction == Driver_pull_out_right || DriverNextAction == Driver_pull_out_left)
                {
                    startCheckForParkingType = true;

                    ChangeFlow(2);
                    EncoderState = 2;
                    cp_counter = 0;
                    pp_counter = 0;
                }
                LOG_ERROR(cString::Format("Starting To Drive"));
                TransmitHeadlight(tTrue,InputTimeStamp);
                Reinitiaze();
            }

            if (m_bDebugModeEnabled)   
                LOG_ERROR(cString::Format("DecisionFilter: DriverModule Data: ID -> %d time: %d isProcessed: %d",DriverNextAction, DriverActionTime,DriverIsProcessed));
        }
        else if(pSource == &m_iWheelEncoderDistance)
        {
            WheelEncoderDistance = 0.0;  
            tUInt32 WheelEncoderTime = 0;
            {   // focus for sample read lock
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(m_pDescriptionOutputSampleDistance,pMediaSample,pCoderInput);       
                // get IDs
                if(!WheelEncoderModuleBufferReaderFlag)
                {
                    pCoderInput->GetID("f32Value", WheelEncoderDistanceBuffer);
                    pCoderInput->GetID("ui32ArduinoTimestamp",WheelEncoderDistanceTimeBuffer);
                    WheelEncoderModuleBufferReaderFlag = tTrue;
                }  
                // get the values from DriverNextAction
                pCoderInput->Get(WheelEncoderDistanceBuffer, (tVoid*)&WheelEncoderDistance);
                pCoderInput->Get(WheelEncoderDistanceTimeBuffer, (tVoid*)&WheelEncoderTime);   
            }

            if(startDriving && execFlow == 2) // 2 == wheelEncoder
            {
                MakeDecision();    
            }   
        }
        else if(pSource==&m_iObjectFilter)
        {
            
            // read-out the incoming Media Sample
            {
                __adtf_sample_read_lock_mediadescription(m_pDescriptionOutputObject,pMediaSample,pCoderInput);  
                tUInt32 size;        
                pCoderInput->Get("size", (tVoid*)&size);
                Object object_array[size];
                pMediaSample->CopyBufferTo((tVoid*)&object_array, sizeof(Object) * size, sizeof(tUInt32), 0);
                std::vector<Object> Objects(object_array, object_array + sizeof(object_array)/sizeof(Object));    
                ObjectsDetected = Objects; 
                ObjectsDetectedTime = InputTimeStamp;
            }
            
            /*if(m_bDebugModeEnabled){
                    for (unsigned int i = 0; i < ObjectsDetected.size(); i++) {
                        if (ObjectsDetected[i].get_absolute_width() < GetPropertyInt("ObsCR::MaxWidth") && 
                            ObjectsDetected[i].get_absolute_width() > GetPropertyInt("ObsCR::MinWidth") && 
                            ObjectsDetected[i].get_absolute_height() < GetPropertyInt("ObsCR::MaxHeight") && 
                            ObjectsDetected[i].get_absolute_height() > GetPropertyInt("ObsCR::MinHeight"))
                            {LOG_ERROR(cString::Format("Object# %d: absCordinates-> [%d , %d] - [ W, H ]-> [%d , %d]",i,ObjectsDetected[i].get_absolute_x(),ObjectsDetected[i].get_absolute_y(),ObjectsDetected[i].get_absolute_width(),ObjectsDetected[i].get_absolute_height()));}
                        else
                            if(m_bDebugModeEnabled) LOG_ERROR(cString::Format("Ignored Object# %d: absCordinates-> [%d , %d] - [ W, H ]-> [%d , %d]",i,ObjectsDetected[i].get_absolute_x(),ObjectsDetected[i].get_absolute_y(),ObjectsDetected[i].get_absolute_width(),ObjectsDetected[i].get_absolute_height()));
                    }
                }*/
                if(!startDriving) RETURN_NOERROR;
        }
        else if(pSource==&m_iSensorFilter)
        {
        
            tFloat32 us_front = 0.0f;
            tFloat32 us_front_left = 0.0f;
            us_front_right = 0.0f;
            us_right = 0.0f;
            
            {   
                __adtf_sample_read_lock_mediadescription(m_pDescriptionOutputSensor,pMediaSample,pCoderInput);       
                // get IDs
                if(!ObstacleSensorModuleBufferReaderFlag)
                {
                    pCoderInput->GetID("minimum_dist[0]", SensorInputFrontBuffer);
                    pCoderInput->GetID("minimum_dist[1]", SensorInputFrontLeftBuffer);
                    pCoderInput->GetID("minimum_dist[2]", SensorInputFrontRightBuffer);
                    pCoderInput->GetID("minimum_dist[4]", SensorInputRightBuffer);
                    ObstacleSensorModuleBufferReaderFlag = tTrue;
                }  
                // get the values from Action Module
                pCoderInput->Get(SensorInputFrontBuffer, (tVoid*)&us_front);
                pCoderInput->Get(SensorInputFrontLeftBuffer, (tVoid*)&us_front_left);   
                pCoderInput->Get(SensorInputFrontRightBuffer, (tVoid*)&us_front_right); 
                pCoderInput->Get(SensorInputRightBuffer, (tVoid*)&us_right);    
            }
            if(!startDriving) RETURN_NOERROR;
             //TODO: Flow based threshholding for US sensor data
            // LOG_ERROR(cString::Format("US Integrity: %d",(us_front != 0 || us_front_left!=0)));
            if (us_front != 0 || us_front_left!=0 || (us_front_right < 0.08 && us_front_right > 0.0))
            {
                FlagObs= true;
            }else
            {
                FlagObs= false;
            }
        }
        else if(pSource==&m_oParkingDetectionInput)
        {
            {   
                __adtf_sample_read_lock_mediadescription(m_pDescParkingDetectionFilter,pMediaSample,pCoderInput);       
                // get IDs
                if(!ParkingDetectionModuleBufferReaderFlag)
                {
                    pCoderInput->GetID("f32Value", DistanceToParkingBuffer);
                    ParkingDetectionModuleBufferReaderFlag = tTrue;
                }  
                // get the values from Action Module
                pCoderInput->Get(DistanceToParkingBuffer, (tVoid*)&DistanceToParking );
             }
             if(!startDriving) RETURN_NOERROR;

                
            if (m_bDebugModeEnabled)   
                LOG_ERROR(cString::Format("Map Filter -> %f",DistanceToParking));
                
                ChangeFlow(2);
                EncoderState = 7;
                Flag_PARKINGAREA = true;
                Flag_ParkingDetection = false; 
        }else if(pSource==&m_oCrossroadDetectionInput)
        {
            float Crossroadvalue = 0.0;
            {   
                __adtf_sample_read_lock_mediadescription(m_pDescCrossroadDetectionFilter,pMediaSample,pCoderInput);       
                // get IDs
                if(!CrossroadDetectionModuleBufferReaderFlag)
                {
                    pCoderInput->GetID("f32Value", CrossRoadDetBuffer);
                    CrossroadDetectionModuleBufferReaderFlag = tTrue;
                }  
                // get the values from Action Module
                pCoderInput->Get(CrossRoadDetBuffer, (tVoid*)&Crossroadvalue );
             }
             if(!startDriving) RETURN_NOERROR;

                
            if (m_bDebugModeEnabled)   
                LOG_ERROR(cString::Format("Map Filter -> %f",Crossroadvalue));
                
            CrossRoadDetected = Crossroadvalue;
        }
        
    }else if (nEventCode == IPinEventSink::PE_MediaTypeChanged && pSource != NULL)
    {
        cObjectPtr<IMediaType> pType;
        pSource->GetMediaType(&pType);
        if (pType != NULL)
        {
            cObjectPtr<IMediaTypeDescription> pMediaTypeDesc;
            RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDesc));
                if(pSource == &m_iLaneTrackingStructInput)
                {
                    m_pDescriptionInputLane = pMediaTypeDesc;
                }
                else if(pSource == &m_iPinRoadSignExt)
                {
                    m_pDescriptionInputMarker = pMediaTypeDesc;   
                }else if(pSource == &m_iDriverAction)
                {
                    m_pDescriptionDecisionInput = pMediaTypeDesc;
                }else if(pSource == &m_iDriverAction)
                {
                    m_pDescriptionOutputSampleDistance = pMediaTypeDesc;
                }else if(pSource == &m_iObjectFilter)
                {
                    m_pDescriptionOutputObject = pMediaTypeDesc;
                }else if(pSource == &m_iSensorFilter)
                {
                    m_pDescriptionOutputSensor = pMediaTypeDesc;
                }
                else if(pSource == &m_oParkingDetectionInput)
                {
                    m_pDescParkingDetectionFilter = pMediaTypeDesc;
                }else if(pSource == &m_oCrossroadDetectionInput)
                {
                    m_pDescCrossroadDetectionFilter = pMediaTypeDesc;
                }
        }        
    }
    RETURN_NOERROR;
}


tResult cDaedalusDecision::MakeDecision()
{
    if(!startDriving) RETURN_NOERROR;
    // if(m_bDebugModeEnabled) {
    //     //LOG_ERROR(cString::Format("Flow: %d, FlagObs: %d",execFlow,FlagObs));
    //     // if(execFlow == 1)
    //     //     LOG_ERROR(cString::Format("Acceleration: %f",LastLaneAceleration));
    //     if(execFlow == 2 && EncoderState == 1)
    //         LOG_ERROR(cString::Format("EncoderState: %d, Acceleration: %f",EncoderState, LastLaneAceleration));
    // }
    if(FlagObs == true && !(EncoderState == 7 && execFlow == 2))
    {
        DriveSlowAfterTurn = GetPropertyInt("slowIterationsAfterTurn");
        Flag_PEDESTRIANCROSSING = tFalse;        
        TransmitAcceleration(0.0,InputTimeStamp);
    }
    else if(execFlow == 1) //Lanetracking
    {
        if(Flag_ParkingDetection){
            FollowLane(6);
        }
        else if(CurrLane){
            LOG_ERROR(cString::Format("Wrong Lane"));
            FollowLane(5);
            if(!ProcessObstaclesOnSide()){
                LOG_ERROR(cString::Format("Change Lane"));
                ChangeFlow(2);
                EncoderState = 5;
                CurrLane = 0;
            }
        }
        else if(markerInSight > 0)
        {
            Flag_PEDESTRIANCROSSING = tFalse;
            markerInSight--; 
            FollowLane(1);
        }else if (DriveSlowAfterTurn > 0)
        {
            DriveSlowAfterTurn--; 
            FollowLane(3);
        }else if(Flag_PEDESTRIANCROSSING){
            FollowLane(4);
        }else{
            if(!ProcessObstaclesOnLane(0)){
                FollowLane(0);
            }else
                FollowLane(2);
        }
            
    }else if(execFlow == 2) // wheelEncoder
    {
        if(EncoderState == 1){//Folow lane till turning 
            DistanceTraveled = DistanceTraveled + WheelEncoderDistance;
            float Diff = DistanceToFollow - DistanceTraveled;
            if (Diff < GetPropertyFloat("DistanceErrorThreshhold"))
            {
                if(m_bDebugModeEnabled)
                    LOG_ERROR(cString::Format("Car Turning Detected, Diff to Groud Truth %f",Diff));
                EncoderState = 2;
                iter = 0;
                DistanceTraveled = 0.0;
                TransmitDetectCrossroad(false);
            }
            else
            {
                if(CrossRoadDetected == 1){
                    if(m_bDebugModeEnabled)
                        LOG_ERROR(cString::Format("Car Turning Detected, Stopped by crossroadDetector"));
                    EncoderState = 2;
                    iter = 0;
                    CrossRoadDetected = 0;
                    DistanceTraveled = 0.0;
                }else if(Flag_HAVEWAY && DriverNextAction == Driver_straight){
                    FollowLane(0); //dont slow
                }else{
                    FollowLane(2);  //do slow
                }
            }
        }else if (EncoderState == 2){//Wait before turning and blinking in parking 
            if(DriverNextAction == Driver_pull_out_left || DriverNextAction == Driver_pull_out_right)
            {
                TransmitHazardIndicator(tTrue,InputTimeStamp);
                TransmitAcceleration(0.0f,InputTimeStamp);
                parkedBlinking++;
                if(parkedBlinking > GetPropertyInt("BlinkingInParkingIter"))
                {
                    TransmitHazardIndicator(tFalse,InputTimeStamp);
                    EncoderState = 7;
                    DistanceTraveled = 0.0;
                    parkedBlinking = 0;   
                    TransmitStartLaneTracking(tFalse,InputTimeStamp);
                    if(m_bDebugModeEnabled)
                        LOG_ERROR(cString::Format("Starting Action: %d",DriverNextAction));
                }

            }else if(Flag_STOPANDGIVEWAY == tTrue || Flag_GIVEWAY == tTrue || Flag_UNMARKEDINTERSECTION == tTrue)
            {
                if(iter < GetPropertyInt("StoppingIterations")){
                    //filterout Obs if Obs iter --
                    if(ProcessObstaclesOnCrossroad() && DriverNextAction != Driver_right) iter--;
                    iter ++;
                    TransmitAcceleration(0.0f, InputTimeStamp);
                }else {
                    EncoderState = 3;
                    DistanceTraveled = 0.0;
                    if(m_bDebugModeEnabled)
                        LOG_ERROR(cString::Format("Starting Action: %d",DriverNextAction));
                    TransmitStartLaneTracking(tFalse,InputTimeStamp);
                }   
            }else
            {
                if(Flag_HAVEWAY && DriverNextAction == Driver_straight)
                    st_straightSpeed = LastLaneAceleration;
                //increase speed straight command here
                EncoderState = 3;
                DistanceTraveled = 0.0;
                TransmitStartLaneTracking(tFalse,InputTimeStamp);
                
            }
        }else if(EncoderState == 3){//Run Action 
            if(DriverNextAction != Driver_straight &&
                DriverNextAction != Driver_right &&
                DriverNextAction != Driver_left)
                {  
                    TransmitAcceleration(0.0f,InputTimeStamp);
                    LOG_ERROR("Jury Command Integrity Failed: Stopping!!");
                    RETURN_NOERROR;
                }
            RunAction(DriverNextAction);
            DistanceTraveled = DistanceTraveled + WheelEncoderDistance;
        }else if(EncoderState == 4){//Wait befor Change Lane 
            TransmitAcceleration(0.0f, InputTimeStamp);
            if (!ProcessObstaclesOnLane(1))
            {
                ChangeFlow(1);
                LOG_ERROR(cString::Format("Obstacle vanish"));
            }
            // LOG_ERROR(cString::Format("Waiting for obstacle to vanish"));
            waitBeforeChangeLane++;
            if(waitBeforeChangeLane > GetPropertyInt("waitBeforeChangeLane"))
            {
                waitBeforeChangeLane = 0;
                EncoderState = 6;
                DistanceTraveled = 0.0;
                if(m_bDebugModeEnabled)
                    LOG_ERROR(cString::Format("Starting Action: Change Lane"));
            }
        }else if(EncoderState == 5){//Return to correct lane 
            RunAction(Driver_change_lane_right);
            DistanceTraveled = DistanceTraveled + WheelEncoderDistance;
        }else if(EncoderState == 6){//Change lane 
            RunAction(Driver_change_lane_left);
            DistanceTraveled = DistanceTraveled + WheelEncoderDistance;
        }else if (EncoderState == 7){//Perform Parking 
            if(DriverNextAction != Driver_parallel_parking &&
                DriverNextAction != Driver_cross_parking &&
                DriverNextAction != Driver_pull_out_right &&
                DriverNextAction != Driver_pull_out_left )
                {  
                    TransmitAcceleration(0.0f,InputTimeStamp);
                    LOG_ERROR("Jury Command Integrity Failed: Stopping!!");
                    RETURN_NOERROR;
                }
            RunAction(DriverNextAction);
            DistanceTraveled = DistanceTraveled + WheelEncoderDistance;
            //TransmitAcceleration(0.0f,InputTimeStamp);
        }else if (EncoderState == 8){//Perform PullingOut 
            TransmitAcceleration(0.0f,InputTimeStamp);
            if(DriverNextAction == Driver_pull_out_right || DriverNextAction == Driver_pull_out_left)
                {
                    EncoderState = 2;
                }
            if(m_bDebugModeEnabled) LOG_ERROR("Stopping after Parking");
        }else if (EncoderState == 0){
            LOG_ERROR(cString::Format("Logical Error"));
        }
    }    
    RETURN_NOERROR;
}

tResult cDaedalusDecision::ProcessObstaclesOnSide()
{
    int count = 0;
    if(ObjectsDetected.size() <= 0)
        count = 0;
    else{
        count = ObjectsDetected.size();
        for(unsigned int i = 0; i < ObjectsDetected.size(); i++){
            if (ObjectsDetected[i].get_absolute_x() > GetPropertyInt("ObsReturn::RightOffset"))
                {
                    count--;
                }
        }
    }    // LOG_ERROR(cString::Format("Returning : Object# %d: Ultra-> [%f , %f]",count,us_right,us_front_right));
    
    if((us_front_right <= 0.03 || us_front_right > GetPropertyFloat("ObsReturn::RightThresh")) 
    && (us_right <= 0.03 || us_right > GetPropertyFloat("ObsReturn::RightThresh") )
    && count == 0 && LastLaneAceleration != 0.0){
        Count_BackToLane++;
        if(Count_BackToLane > GetPropertyInt("LaneReturnIterations")){
            Count_BackToLane = 0;
            return 0;
        }
    }
    return 1;
}

tResult cDaedalusDecision::ProcessObstaclesOnLane(bool detected){
    if(CurrLane || ObjectsDetected.size() <= 0 || (LastLaneSteerPT1 > 98 || LastLaneSteerPT1 < 82) || Flag_NOOVERTAKING){
        LastDetTime_MoveAroundObs = 0;
        CountMoveAroundObs = 0;
        return 0;
    }
    
    int count = ObjectsDetected.size();
    for(unsigned int i = 0; i < ObjectsDetected.size(); i++){
        int startPoint = ObjectsDetected[i].get_absolute_x();
        int endPoint = startPoint + ObjectsDetected[i].get_absolute_width();
        if ( startPoint < 0 ||
            (endPoint < GetPropertyInt("ObsLane::LeftOffset") ||
                startPoint > GetPropertyInt("ObsLane::RightOffset"))  ||
            ObjectsDetected[i].get_absolute_width() > GetPropertyInt("ObsLane::MaxWidth") ||
                ObjectsDetected[i].get_absolute_width() < GetPropertyInt("ObsLane::MinWidth")  )
            {
                count--;
                 //if(m_bDebugModeEnabled) LOG_ERROR(cString::Format("Ignored Object# %d: absCordinates-> [%d , %d] - [ Width, Height ]-> [%d , %d]",i,ObjectsDetected[i].get_absolute_y(),ObjectsDetected[i].get_absolute_y(),ObjectsDetected[i].get_absolute_width(),ObjectsDetected[i].get_absolute_height()));
            }
        else
            if(m_bDebugModeEnabled) LOG_ERROR(cString::Format("Object# %d: absCordinates-> [%d , %d] - [ Width, Height ]-> [%d , %d]",i,ObjectsDetected[i].get_absolute_x(),ObjectsDetected[i].get_absolute_y(),ObjectsDetected[i].get_absolute_width(),ObjectsDetected[i].get_absolute_height()));
    }
    if (detected){
        return count;
    }
    if (count == 1){
        int diff = ObjectsDetectedTime - LastDetTime_MoveAroundObs;
        if (diff < GetPropertyInt("MarkerTime_Threshold") || LastDetTime_MoveAroundObs==0)
        {
            CountMoveAroundObs++;
            LastDetTime_MoveAroundObs=ObjectsDetectedTime;
        }else
        {
            LastDetTime_MoveAroundObs = 0;
            CountMoveAroundObs = 0;
            return 0;
        }
        if (CountMoveAroundObs >= GetPropertyInt("CountObjectDetection") )
        {
            ChangeFlow(2);
            EncoderState = 4;
            DistanceTraveled = 0;
            
        }
    }else{
        LastDetTime_MoveAroundObs = 0;
        CountMoveAroundObs = 0;
        return 0;
    } 
    return count;
}

tResult cDaedalusDecision::ProcessObstaclesOnCrossroad(){
    if(ObjectsDetected.size() <= 0)
        return 0;
    int  count = 0; 
    for(unsigned int i = 0; i < ObjectsDetected.size(); i++){
        if (ObjectsDetected[i].get_absolute_width() < GetPropertyInt("ObsCR::MaxWidth") && 
            ObjectsDetected[i].get_absolute_width() > GetPropertyInt("ObsCR::MinWidth") && 
            ObjectsDetected[i].get_absolute_height() < GetPropertyInt("ObsCR::MaxHeight") && 
            ObjectsDetected[i].get_absolute_height() > GetPropertyInt("ObsCR::MinHeight"))
            {
                count++;
                if(m_bDebugModeEnabled) LOG_ERROR(cString::Format("Object# %d: absCordinates-> [%d , %d] - [ Width, Height ]-> [%d , %d]",i,ObjectsDetected[i].get_absolute_x(),ObjectsDetected[i].get_absolute_y(),ObjectsDetected[i].get_absolute_width(),ObjectsDetected[i].get_absolute_height()));
            }
        // ObjectsDetected[i].get_absolute_height();
    }
    return count;
}


void cDaedalusDecision::Stop(){
    LOG_ERROR(cString::Format("Stopping car"));
    TransmitStartLaneTracking(false,InputTimeStamp);
    TransmitHeadlight(tFalse,InputTimeStamp);
    TransmitAcceleration(0.0f,InputTimeStamp);
    TransmitLeftIndicator(tFalse,InputTimeStamp);
    TransmitHazardIndicator(tFalse,InputTimeStamp);
    TransmitRightIndicator(tFalse,InputTimeStamp);
    TransmitReverseIndicator(tFalse,InputTimeStamp);
    CrossRoadDetected = 0;
    Count_BackToLane = 0;
    us_front = 0.0f;
    us_front_left = 0.0f;
    us_front_right = 0.0f;
    us_right = 0.0f;
    waitBeforeChangeLane=0;
    ObjectsDetectedTime = 0;
    CurrLane = 0;
    startDriving = false;
    Flag_UNMARKEDINTERSECTION=tFalse;
    Flag_STOPANDGIVEWAY=tFalse;
    Flag_PARKINGAREA=tFalse;
    Flag_HAVEWAY=tFalse;
    Flag_GIVEWAY=tFalse;
    Flag_PARKINGAREA = tFalse;
    Flag_ParkingDetection = false;
    // Flag_ONEWAYSTREET=tFalse;
    Flag_NOMATCH=tTrue;
    Count_UNMARKEDINTERSECTION = 0;
    Count_STOPANDGIVEWAY=0;
    Count_PARKINGAREA=0;
    Count_HAVEWAY=0;
    Count_GIVEWAY=0;
    Count_NOOVERTAKING=0;
    CountMoveAroundObs=0;
    LastDetTime_MoveAroundObs = 0;
    LastDetTime_UNMARKEDINTERSECTION = 0;
    LastDetTime_STOPANDGIVEWAY=0;
    LastDetTime_PARKINGAREA=0;
    LastDetTime_HAVEWAY=0;
    LastDetTime_GIVEWAY=0;
    FlagObs = false;
    DistanceToFollow = 0.0f;
    DistanceTraveled = 0.0f;
    DriverNextAction = -1;
    iter=0;
    ChangeFlow(1);
    EncoderState = 0;
    parkedBlinking = 0;
}


void cDaedalusDecision::Reinitiaze(){ //TODO : FIX Variables here
    startDriving = true;
    TransmitStartLaneTracking(tTrue,InputTimeStamp);
    TransmitHeadlight(tTrue,InputTimeStamp);
}

void cDaedalusDecision::ChangeFlow(int flow)
{
    if(m_bDebugModeEnabled)
        LOG_ERROR(cString::Format("changing flow to %d",flow));
    execFlow = flow;
    Flag_PEDESTRIANCROSSING = tFalse;
    DistanceTraveled = 0;
    EncoderState = 1;
}

void cDaedalusDecision::ActionCompleted(bool TellJury)
{
    if(m_bDebugModeEnabled)
        LOG_ERROR(cString::Format("Action Completed"));
    startCheckForParkingType = false;
    Flag_NOMATCH = tTrue;
    Flag_UNMARKEDINTERSECTION = tFalse;
    CrossRoadDetected = 0;
    Flag_STOPANDGIVEWAY = tFalse;
    Flag_HAVEWAY = tFalse;
    Flag_GIVEWAY = tFalse;
    Flag_NOOVERTAKING = tFalse;
    //DriverNextAction = -1;
    //TransmitFeedbackToDriverModule();
    prevDriverAction = DriverNextAction;
    if (prevDriverAction == Driver_parallel_parking || prevDriverAction == Driver_cross_parking )
    {
        if(m_bDebugModeEnabled) LOG_ERROR("Will not follow lane.");
        ChangeFlow(2);
        EncoderState = 8;
    }else{
        if(m_bDebugModeEnabled) LOG_ERROR("Will follow lane.");
        ChangeFlow(1);
        DriveSlowAfterTurn = GetPropertyInt("slowIterationsAfterTurn");
        TransmitStartLaneTracking(tTrue,m_tsArduinoTime);
    }    
    if(TellJury){
        DriverNextAction = -1;
        TransmitFeedbackToDriverModule();
        
    }
    TransmitAcceleration(0.0f,m_tsArduinoTime);
    TransmitSteer(90.0,m_tsArduinoTime);
    TransmitHazardIndicator(tFalse,m_tsArduinoTime);
    TransmitReverseIndicator(tFalse,m_tsArduinoTime);
    TransmitRightIndicator(tFalse,m_tsArduinoTime);
    TransmitLeftIndicator(tFalse,m_tsArduinoTime);
    st_straightSpeed = GetPropertyFloat(AF_PROP_STRAIGHT_SPEED); 
}

//// Action filter methods
tVoid cDaedalusDecision::InitializeProperties()
{   
    prevDriverAction = -1;
    startLaneTracking = false;
    m_bFirstFrame = true;
    m_ui8InitCtrl = 0;
    startCheckForParkingType = false;
}

tResult cDaedalusDecision::ReadProperties()
{

    total_counter = GetPropertyInt("ParkingTypeDectectionIteration");
    lastCommand = GetPropertyInt(AF_PROP_LAST_COMMAND);
    cameraOffset = GetPropertyInt(AF_PROP_PARKING_CAMERA_OFFSET);
    GetChangeLaneProperties();
    GetCrossParkingProperties();
    GetParallelParkingProperties();
    GetPullOverLeftProperties();
    GetPullOverRightProperties();
    GetPullOverLeftFromParallelParkingProperties();
    GetPullOverRightFromParallelParkingProperties();
    GetLeftTurnProperties();
    GetRightTurnProperties();
    GetStraightProperties();
    
    if(lastCommand != -1)
    {
        prevDriverAction = lastCommand;
        
    }
    if(m_bDebugModeEnabled)
    {
        LOG_ERROR(cString::Format("prevDriverAction %d", prevDriverAction));
    }
    RETURN_NOERROR;
}

tResult cDaedalusDecision::ProcessForFirstFrame()
{
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
    RETURN_NOERROR;
}

tResult cDaedalusDecision::ProcessInput(IMediaSample* pSample, tTimeStamp tsInputTime)
{
    if(startCheckForParkingType)
    {

        if(m_ui8InitCtrl < 150)         // This loop is necessary to boot the car's controller (needs neutral signal for a certain time)
        {
            m_ui8InitCtrl++;
        }
        else
        {

            if (m_bDebugModeEnabled)
            {
               LOG_ERROR(cString::Format("Video input started"));
            }

            RETURN_IF_POINTER_NULL(pSample);
            const tVoid* l_pSrcBuffer;
            IplImage* oImg = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
            RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
            oImg->imageData = (char*)l_pSrcBuffer;
            Mat image(cvarrToMat(oImg));
            cvReleaseImage(&oImg);
            pSample->Unlock(l_pSrcBuffer);
            if( (pp_counter + cp_counter) >= total_counter)
            {
                startCheckForParkingType = false;
                RunAction(DriverNextAction);
            }
            // TODO : MAKE A CAMERA OFFEST PROPERTY
            int result = parkingType.CrossedParking(image, 10);
            if(FlagObs)
            {
                pp_counter++;
            }
            if  ( result == Driver_cross_parking)
            {

                if (m_bDebugModeEnabled)
                {
                   LOG_ERROR(cString::Format("Driver_cross_parking"));
                }
                /*if(m_bDebugModeEnabled)
                {
                    LOG_ERROR(cString::Format("Saving Image"));
                    String imgPath = "/home/aadc/Desktop/images/" + patch::to_string(imageCount++) + ".png" ;
                    imwrite(imgPath, inputImage);
                }*/
                cp_counter++;
               
            }
            else if (result == Driver_parallel_parking)
            {
                if (m_bDebugModeEnabled)
                {
                   LOG_ERROR(cString::Format("Driver_parallel_parking"));
                }

                pp_counter++;
            }
            else 
            {
                // TODO: By If Detect nothing which parking to choose
                pp_counter++;
                if (m_bDebugModeEnabled)
                {
                   LOG_ERROR(cString::Format("Nothing found"));
                }
            }
        }
    }
    RETURN_NOERROR;            
}


/****************** Properties Setter Methods **********************/
void cDaedalusDecision::SetAllProperties()
{
   /****************** Select Type of Parking ***************/
    SetPropertyInt(AF_PROP_LAST_COMMAND,-1);
    SetPropertyStr(AF_PROP_LAST_COMMAND NSSUBPROP_DESCRIPTION, "Number of last parking type");
    SetPropertyInt(AF_PROP_PARKING_CAMERA_OFFSET,0);
    /***************** End of Debug *****************/

    SetChangeLaneProperties();
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
}

void cDaedalusDecision::SetChangeLaneProperties()
{
    /************* Change Lane Left Properties *******************/
    SetPropertyFloat(AF_PROP_CL_BACK_SPEED,-0.7);
    SetPropertyFloat(AF_PROP_CL_LEFT_SPEED,0.7);
    SetPropertyFloat(AF_PROP_CL_RIGHT_SPEED,0.7);
    SetPropertyFloat(AF_PROP_CL_FORWARD_SPEED,0.0);

    SetPropertyFloat(AF_PROP_CL_BACK_ANGLE,90.0);
    SetPropertyFloat(AF_PROP_CL_LEFT_ANGLE,60.0);
    SetPropertyFloat(AF_PROP_CL_FORWARD_ANGLE,90.0);
    SetPropertyFloat(AF_PROP_CL_RIGHT_ANGLE,120.0);

    SetPropertyFloat(AF_PROP_CL_BACK_DURATION,0.0);
    SetPropertyFloat(AF_PROP_CL_LEFT_DURATION,0.4);
    SetPropertyFloat(AF_PROP_CL_FORWARD_DURATION,0.0);
    SetPropertyFloat(AF_PROP_CL_RIGHT_DURATION,0.4);
}

void cDaedalusDecision::SetParallelParkingProperties()
{
    /************* Parallel Parking Properties *******************/
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_ANGLE,120.0);
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_RIGHT_ANGLE,60.0);
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_LEFT_ANGLE,60.0);
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_BACKWARD_ANGLE,120.0);
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_LEFT_ANGLE,60.0);
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_LAST_ANGLE,120.0);
    
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_LEFT_SPEED,-0.5);
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_SPEED,0.5);
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_RIGHT_SPEED,-0.5);
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_LEFT_SPEED,0.5);
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_BACKWARD_SPEED,-0.5);
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_FORWARD_LAST_SPEED,-0.5);

    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_FORWARD,0.55);
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_LEFT_TURNING,0.12);
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_RIGHT_TURNING,0.75);
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_BACKWARD,0);
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_FORWARD_LEFT,0.5);
    SetPropertyFloat(AF_PROP_PARALLEL_PARKING_DURATION_FORWARD_LAST,0.0);
}

void cDaedalusDecision::SetCrossParkingProperties()
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


    SetPropertyFloat(AF_PROP_CROSS_PARKING_DURATION_FORWARD,0.35);
    SetPropertyFloat(AF_PROP_CROSS_PARKING_DURATION_LEFT_TURNING,0.4);
    SetPropertyFloat(AF_PROP_CROSS_PARKING_DURATION_RIGHT_TURNING,0.45);
    SetPropertyFloat(AF_PROP_CROSS_PARKING_DURATION_BACKWARD,0.35);

}

void cDaedalusDecision::SetStraightProperties()
{
    /************* Straight Properties *****************/
    SetPropertyFloat(AF_PROP_STRAIGHT_SPEED,1.3);
    SetPropertyFloat(AF_PROP_STRAIGHT_DURATION_FORWARD,0.8);
    SetPropertyFloat(AF_PROP_STRAIGHT_STRAIGHT_ANGLE,90.0);
}

void cDaedalusDecision::SetLeftTurnProperties()
{
    /************* Turn Left Properties ****************/
    SetPropertyFloat(AF_PROP_LEFT_ANGLE,60.0);
    SetPropertyFloat(AF_PROP_LEFT_DURATION_FORWARD,0.48);
    SetPropertyFloat(AF_PROP_LEFT_DURATION_TURNING,1.2);
    SetPropertyFloat(AF_PROP_LEFT_SPEED,0.8);
    SetPropertyFloat(AF_PROP_LEFT_STRAIGHT_ANGLE,90.0);
    SetPropertyFloat(AF_PROP_LEFT_STRAIGHT_SPEED,1.0);

    SetPropertyFloat(AF_PROP_LEFT_END_ANGLE,90.0);
    SetPropertyFloat(AF_PROP_LEFT_END_DURATION,0.0);
    SetPropertyFloat(AF_PROP_LEFT_END_SPEED,1.3);

    SetPropertyBool(AF_PROP_LEFT_ANGLE NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(AF_PROP_LEFT_ANGLE NSSUBPROP_DESCRIPTION, "Angle for left turn");
}

void cDaedalusDecision::SetRightTurnProperties()
{
    /************* Turn Right Properties *******************/
    SetPropertyFloat(AF_PROP_RIGHT_ANGLE,120.0);
    SetPropertyFloat(AF_PROP_RIGHT_DURATION_FORWARD,0.21);
    SetPropertyFloat(AF_PROP_RIGHT_DURATION_TURNING,1.24);
    SetPropertyFloat(AF_PROP_RIGHT_SPEED,0.9);

    SetPropertyFloat(AF_PROP_RIGHT_STRAIGHT_ANGLE,90.0);
    SetPropertyFloat(AF_PROP_RIGHT_STRAIGHT_SPEED,0.8);

}

void cDaedalusDecision::SetPullOverLeftProperties()
{
    /************* Pull Over left Properties *******************/
    SetPropertyFloat(AF_PROP_PL_LEFT_SPEED,0.7);
    SetPropertyFloat(AF_PROP_PL_LEFT_ANGLE,60.0);
    SetPropertyFloat(AF_PROP_PL_LEFT_DURATION,1.0);

    SetPropertyFloat(AF_PROP_PL_LEFT_STRAIGHT_SPEED,0.7);
    SetPropertyFloat(AF_PROP_PL_LEFT_STRAIGHT_ANGLE,90.0);
    SetPropertyFloat(AF_PROP_PL_LEFT_DURATION_STRAIGHT,0.3);
}

void cDaedalusDecision::SetPullOverRightProperties()
{
    /************* Pull Over Right Properties *******************/
    SetPropertyFloat(AF_PROP_PR_RIGHT_SPEED,0.7);
    SetPropertyFloat(AF_PROP_PR_RIGHT_ANGLE,120.0);
    SetPropertyFloat(AF_PROP_PR_RIGHT_DURATION,1.2);

    SetPropertyFloat(AF_PROP_PR_RIGHT_STRAIGHT_SPEED,0.7);
    SetPropertyFloat(AF_PROP_PR_RIGHT_STRAIGHT_ANGLE,90.0);
    SetPropertyFloat(AF_PROP_PR_RIGHT_DURATION_STRAIGHT,0.1);
}

tVoid cDaedalusDecision::SetPullOverRightFromParallelParkingProperties()
{
    /************* Pull Over Parallel Right Properties *******************/
    SetPropertyFloat(AF_PROP_PR_PP_BACK_SPEED,0.4);
    SetPropertyFloat(AF_PROP_PR_PP_LEFT_SPEED,-0.5);
    SetPropertyFloat(AF_PROP_PR_PP_RIGHT_SPEED,0.5);

    SetPropertyFloat(AF_PROP_PR_PP_BACK_ANGLE,60.0);
    SetPropertyFloat(AF_PROP_PR_PP_LEFT_ANGLE,120.0);
    SetPropertyFloat(AF_PROP_PR_PP_RIGHT_ANGLE,120.0);

    SetPropertyFloat(AF_PROP_PR_PP_BACK_DURATION,0.05);
    SetPropertyFloat(AF_PROP_PR_PP_LEFT_DURATION,0.3);
    SetPropertyFloat(AF_PROP_PR_PP_RIGHT_DURATION,0.65);

    SetPropertyFloat(AF_PROP_PR_PP_FORWARD_SPEED,0.5);//
    SetPropertyFloat(AF_PROP_PR_PP_FORWARD_ANGLE,90.0);
    SetPropertyFloat(AF_PROP_PR_PP_FORWARD_DURATION,0.65);
}

void cDaedalusDecision::SetPullOverLeftFromParallelParkingProperties()
{
    /************* Pull Over Parallel left Properties *******************/
    SetPropertyFloat(AF_PROP_PL_PP_BACK_SPEED,-1.0);
    SetPropertyFloat(AF_PROP_PL_PP_FORWARD_SPEED,1.0);
    SetPropertyFloat(AF_PROP_PL_PP_LEFT_SPEED,1.0);

    SetPropertyFloat(AF_PROP_PL_PP_BACK_ANGLE,120.0);
    SetPropertyFloat(AF_PROP_PL_PP_FORWARD_ANGLE,60.0);
    SetPropertyFloat(AF_PROP_PL_PP_LEFT_ANGLE,60.0);

    SetPropertyFloat(AF_PROP_PL_PP_BACK_DURATION,0.05);
    SetPropertyFloat(AF_PROP_PL_PP_FORWARD_DURATION,0.2);   
    SetPropertyFloat(AF_PROP_PL_PP_LEFT_DURATION,1.2);
    
}

tResult cDaedalusDecision::RunAction(int DriverNextAction)
{
    switch(DriverNextAction)
    {
        case Driver_straight:
            RunStraightScript();
            break;

        case Driver_left:
            RunTurnLeftScript();
            break;
        case Driver_right:
             RunTurnRightScript();
            break;
        case Driver_parallel_parking:
            RunParallelParkingScript();
            break;
        case Driver_cross_parking:
            RunCrossParkingScript();
            break;
        case Driver_pull_out_left:
            RunPullOverLeftScript();
            break;
        case Driver_pull_out_right:
            CheckParkingTypeAndPullOutRight();
            break;
        case Driver_change_lane_left:
            RunChangeLaneLeftScript();
            break;
        case Driver_change_lane_right:
            RunChangeLaneRightScript();
            break;
    }
  RETURN_NOERROR;
}
/****************** Properties Getter Methods **********************/

void cDaedalusDecision::GetChangeLaneProperties()
{
    /************ Change Lane Properties ****************/
    cl_backAngle = GetPropertyFloat(AF_PROP_CL_BACK_ANGLE);
    cl_forwardAngle = GetPropertyFloat(AF_PROP_CL_FORWARD_ANGLE);
    cl_rightAngle = GetPropertyFloat(AF_PROP_CL_RIGHT_ANGLE);
    cl_leftAngle = GetPropertyFloat(AF_PROP_CL_LEFT_ANGLE);

    cl_backSpeed = GetPropertyFloat(AF_PROP_CL_BACK_SPEED);
    cl_forwardSpeed = GetPropertyFloat(AF_PROP_CL_FORWARD_SPEED);
    cl_rightSpeed = GetPropertyFloat(AF_PROP_CL_RIGHT_SPEED);
    cl_leftSpeed = GetPropertyFloat(AF_PROP_CL_LEFT_SPEED);

    cl_backDuration = GetPropertyFloat(AF_PROP_CL_BACK_DURATION);
    cl_forwardDuration = GetPropertyFloat(AF_PROP_CL_FORWARD_DURATION);
    cl_rightDuration = GetPropertyFloat(AF_PROP_CL_RIGHT_DURATION);
    cl_leftDuration = GetPropertyFloat(AF_PROP_CL_LEFT_DURATION);
}

void cDaedalusDecision::GetStraightProperties()
{
    /************ Straight Properties ****************/
    st_straightAngle = GetPropertyFloat(AF_PROP_STRAIGHT_STRAIGHT_ANGLE);
    st_straightDuration = GetPropertyFloat(AF_PROP_STRAIGHT_DURATION_FORWARD);
    st_straightSpeed = GetPropertyFloat(AF_PROP_STRAIGHT_SPEED);
}

void cDaedalusDecision::GetLeftTurnProperties()
{
    /************ Left Turn Properties ****************/
    lt_leftAngle = GetPropertyFloat(AF_PROP_LEFT_ANGLE);
    lt_leftDuration = GetPropertyFloat(AF_PROP_LEFT_DURATION_TURNING);
    lt_leftSpeed = GetPropertyFloat(AF_PROP_LEFT_SPEED);

    lt_straightAngle = GetPropertyFloat(AF_PROP_LEFT_STRAIGHT_ANGLE);
    lt_straightDuration = GetPropertyFloat(AF_PROP_LEFT_DURATION_FORWARD);
    lt_straightSpeed = GetPropertyFloat(AF_PROP_LEFT_STRAIGHT_SPEED);

    lt_endSpeed =  GetPropertyFloat(AF_PROP_LEFT_END_SPEED);
    lt_endDuration =  GetPropertyFloat(AF_PROP_LEFT_END_DURATION);
    lt_endAngle =  GetPropertyFloat(AF_PROP_LEFT_END_ANGLE);
    
}

void cDaedalusDecision::GetRightTurnProperties()
{
    /************ Right Turn Properties ****************/
    rt_rightAngle = GetPropertyFloat(AF_PROP_RIGHT_ANGLE);
    rt_rightDuration = GetPropertyFloat(AF_PROP_RIGHT_DURATION_TURNING);
    rt_rightSpeed = GetPropertyFloat(AF_PROP_RIGHT_SPEED);

    rt_straightAngle = GetPropertyFloat(AF_PROP_RIGHT_STRAIGHT_ANGLE);
    rt_straightDuration = GetPropertyFloat(AF_PROP_RIGHT_DURATION_FORWARD);
    rt_straightSpeed = GetPropertyFloat(AF_PROP_RIGHT_STRAIGHT_SPEED);
}

void cDaedalusDecision::GetCrossParkingProperties()
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

void cDaedalusDecision::GetParallelParkingProperties()
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

void cDaedalusDecision::GetPullOverLeftProperties()
{
    /************* Pull Over Left Properties *******************/
    pl_leftAngle = GetPropertyFloat(AF_PROP_PL_LEFT_ANGLE);
    pl_leftSpeed = GetPropertyFloat(AF_PROP_PL_LEFT_SPEED);
    pl_leftDuration = GetPropertyFloat(AF_PROP_PL_LEFT_DURATION);

    pl_straightAngle = GetPropertyFloat(AF_PROP_PL_LEFT_STRAIGHT_ANGLE);
    pl_straightSpeed = GetPropertyFloat(AF_PROP_PL_LEFT_STRAIGHT_SPEED);
    pl_straightDuration = GetPropertyFloat(AF_PROP_PL_LEFT_DURATION_STRAIGHT);
    
}

void cDaedalusDecision::GetPullOverRightProperties()
{
    /************* Pull Over Right Properties *******************/
    pr_rightAngle = GetPropertyFloat(AF_PROP_PR_RIGHT_ANGLE);
    pr_rightSpeed = GetPropertyFloat(AF_PROP_PR_RIGHT_SPEED);
    pr_rightDuration = GetPropertyFloat(AF_PROP_PR_RIGHT_DURATION);

    pr_straightAngle = GetPropertyFloat(AF_PROP_PR_RIGHT_STRAIGHT_ANGLE);
    pr_straightSpeed = GetPropertyFloat(AF_PROP_PR_RIGHT_STRAIGHT_SPEED);
    pr_straightDuration = GetPropertyFloat(AF_PROP_PR_RIGHT_DURATION_STRAIGHT);
}

void cDaedalusDecision::GetPullOverRightFromParallelParkingProperties()
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

    pr_pp_forwardSpeed = GetPropertyFloat(AF_PROP_PR_PP_FORWARD_SPEED);
    pr_pp_forwardAngle = GetPropertyFloat(AF_PROP_PR_PP_FORWARD_ANGLE);
    pr_pp_forwardDuration = GetPropertyFloat(AF_PROP_PR_PP_FORWARD_DURATION);
}

void cDaedalusDecision::GetPullOverLeftFromParallelParkingProperties()
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

void cDaedalusDecision::CheckParkingTypeAndPullOutRight()
{
    if(prevDriverAction == Driver_cross_parking)
    {
        RunPullOverRightScript();
    }
    else if(prevDriverAction == Driver_parallel_parking)
    {
        RunPullOverRightFromParallelParkingScript();
    }
   
    else if( (pp_counter + cp_counter) == total_counter)
    {
        startCheckForParkingType = false;
        if(cp_counter > pp_counter)
        {
           
            RunPullOverRightScript();
        }
        else
        {
            RunPullOverRightFromParallelParkingScript();
        }
    }

}

/********************* Scripts Methods *****************************/
void cDaedalusDecision::RunChangeLaneLeftScript()
{
    /************ GetChangeLaneProperties Properties *******************/
    // if(m_bDebugModeEnabled)
    // {
    //     LOG_ERROR(cString::Format("RunChangeLaneLeftScript: Started"));
    // }
    
    TransmitLeftIndicator(tTrue,m_tsArduinoTime);
    TransmitReverseIndicator(tTrue,m_tsArduinoTime);
    if (DistanceTraveled < ( cl_leftDuration ))
    {
        TransmitReverseIndicator(tFalse,m_tsArduinoTime);
        // if(m_bDebugModeEnabled)
        // {
        //     LOG_ERROR(cString::Format("RunChangeLaneLeftScript: Moving Left"));
        // }
        TransmitAcceleration(cl_leftSpeed,m_tsArduinoTime);
        TransmitSteer(cl_leftAngle,m_tsArduinoTime);
         
    } 
    else if (DistanceTraveled < ( cl_leftDuration + cl_forwardDuration ))
    {
       
        // if(m_bDebugModeEnabled)
        // {
        //     LOG_ERROR(cString::Format("RunChangeLaneLeftScript: Moving Forward"));
        // }
        TransmitAcceleration(cl_forwardSpeed,m_tsArduinoTime);
        TransmitSteer(cl_forwardAngle,m_tsArduinoTime);
    } 
    else
    {
        CurrLane = 1;
        ActionCompleted(0);

    }
}

void cDaedalusDecision::RunChangeLaneRightScript()
{
    /************ GetChangeLaneProperties Properties *******************/
    // if(m_bDebugModeEnabled)
    // {
    //     LOG_ERROR(cString::Format("RunChangeLaneRightScript: Started"));
    // }
    TransmitRightIndicator(tTrue,m_tsArduinoTime);
    if (DistanceTraveled < ( cl_rightDuration ))
    {
        // if(m_bDebugModeEnabled)
        // {
        //     LOG_ERROR(cString::Format("RunChangeLaneRightScript: Moving Right"));
        // }
        TransmitAcceleration(cl_rightSpeed,m_tsArduinoTime);
        TransmitSteer(cl_rightAngle,m_tsArduinoTime);
         
    } 
    else if (DistanceTraveled < ( cl_rightDuration +  cl_backDuration ))
    {
       
        // if(m_bDebugModeEnabled)
        // {
        //     LOG_ERROR(cString::Format("RunChangeLaneRightScript: Moving Forward"));
        // }
        TransmitAcceleration(cl_backSpeed,m_tsArduinoTime);
        TransmitSteer(cl_backAngle,m_tsArduinoTime);
    } 
    else
    {
        ActionCompleted(0);
        CurrLane=0;
    }
}

void cDaedalusDecision::RunTurnLeftScript()
{
    /************ Turn Left Properties *******************/
    // if(m_bDebugModeEnabled)
    // {
    //     // LOG_ERROR(cString::Format("RunTurnLeftScript: Started"));
    // }
    
    TransmitLeftIndicator(tTrue,m_tsArduinoTime);
    
    if (DistanceTraveled <  lt_straightDuration)
    {
        // if(m_bDebugModeEnabled)
        // {
        //     LOG_ERROR(cString::Format("RunTurnLeftScript: Moving Straight"));
        // }
        TransmitAcceleration(lt_straightSpeed,m_tsArduinoTime);
        TransmitSteer(lt_straightAngle,m_tsArduinoTime);
    } 
    else if (DistanceTraveled < ( lt_straightDuration + lt_leftDuration ))
    {
        // if(m_bDebugModeEnabled)
        // {
        //     LOG_ERROR(cString::Format("RunTurnLeftScript: Moving Left"));
        // }
        TransmitAcceleration(lt_leftSpeed,m_tsArduinoTime);
        TransmitSteer(lt_leftAngle,m_tsArduinoTime);
    } 
    else if (DistanceTraveled < ( lt_straightDuration + lt_leftDuration + lt_endDuration ))
    {
       
        // if(m_bDebugModeEnabled)
        // {
        //     LOG_ERROR(cString::Format("RunTurnLeftScript: Moving Forward"));
        // }
        TransmitAcceleration(lt_endSpeed,m_tsArduinoTime);
        TransmitSteer(lt_endAngle,m_tsArduinoTime);
    } 
    else
    {
        ActionCompleted(1);
    }
}

void cDaedalusDecision::RunTurnRightScript()
{
    /************ Turn Right Properties *******************/
    // if(m_bDebugModeEnabled)
    // {
    //     LOG_ERROR(cString::Format("RunTurnRightScript: Start"));
    // }
    TransmitRightIndicator(tTrue,m_tsArduinoTime);

    if (DistanceTraveled < rt_straightDuration )
    {
        // if(m_bDebugModeEnabled)
        // {
        //     LOG_ERROR(cString::Format("RunTurnRightScript: Moving Straight"));
        // }
        TransmitAcceleration(rt_straightSpeed,m_tsArduinoTime);
        TransmitSteer(rt_straightAngle,m_tsArduinoTime);
    } 
    else if (DistanceTraveled < (  rt_straightDuration + rt_rightDuration) )
    {

        // if(m_bDebugModeEnabled)
        // {
        //     LOG_ERROR(cString::Format("RunTurnRightScript: Moving Right"));
        // }
        TransmitAcceleration(rt_rightSpeed,m_tsArduinoTime);
        TransmitSteer(rt_rightAngle,m_tsArduinoTime);
    } 
    else 
    {
        ActionCompleted(1);
    }

}

void cDaedalusDecision::RunStraightScript()
{
    /************ Straight Properties *******************/
    // if(m_bDebugModeEnabled)
    // {
    //     LOG_ERROR(cString::Format("Action Fiter: In straight : %f , Distance Travelled : %f",st_straightDuration,DistanceTraveled));
    // }
    if (DistanceTraveled <  st_straightDuration )
    {
        TransmitAcceleration(st_straightSpeed,m_tsArduinoTime);
        TransmitSteer(st_straightAngle,m_tsArduinoTime);
    } 
    else 
    {
        ActionCompleted(1);
    }
}

void cDaedalusDecision::RunCrossParkingScript()
{
    /************ Cross Parking Properties *******************/
    TransmitRightIndicator(tTrue,m_tsArduinoTime);
    // if(m_bDebugModeEnabled)
    // {
    //     LOG_ERROR(cString::Format("RunCrossParkingScript: Cross Parking Started"));
    // }
    if (DistanceTraveled <  cp_straightDuration )
    {
        TransmitAcceleration(cp_straightSpeed,m_tsArduinoTime);
        TransmitSteer(cp_straightAngle,m_tsArduinoTime);
    } 
    else if (DistanceTraveled < ( cp_straightDuration + cp_leftDuration) )
    {
        TransmitAcceleration(cp_leftSpeed,m_tsArduinoTime);
        TransmitSteer(cp_leftAngle,m_tsArduinoTime);
    }
    else if (DistanceTraveled <( cp_straightDuration + cp_leftDuration + cp_rightDuration) )
    {
        //TransmitHazardIndicator(tTrue,m_tsArduinoTime);
        //TransmitReverseIndicator(tTrue,m_tsArduinoTime);
        TransmitAcceleration(cp_rightSpeed,m_tsArduinoTime);
        TransmitSteer(cp_rightAngle,m_tsArduinoTime);
    }
    else if (DistanceTraveled < ( cp_straightDuration + cp_leftDuration + cp_rightDuration + cp_backDuration))
    {
        //TransmitHazardIndicator(tTrue,m_tsArduinoTime);
    //  TransmitReverseIndicator(tTrue,m_tsArduinoTime);
        TransmitAcceleration(cp_backSpeed,m_tsArduinoTime);
        TransmitSteer(cp_backAngle,m_tsArduinoTime);
    }
    else 
    {
        ActionCompleted(1);
    }
}

void cDaedalusDecision::RunParallelParkingScript()
{
    /************ Parallel Parking Properties *******************/
    TransmitRightIndicator(tTrue,m_tsArduinoTime);
    if (DistanceTraveled <  pp_straightDuration )
    {
        TransmitAcceleration(pp_straightSpeed,m_tsArduinoTime);
        TransmitSteer(pp_straightAngle,m_tsArduinoTime);
    } 
    else if (DistanceTraveled < ( pp_straightDuration + pp_forwardLeftDuration) )
    {
        TransmitAcceleration(pp_forwardLeftSpeed ,m_tsArduinoTime);
        TransmitSteer(pp_forwardLeftAngle,m_tsArduinoTime);
    }
    else if (DistanceTraveled < ( pp_straightDuration + pp_forwardLeftDuration + pp_rightDuration) )
    {
        TransmitAcceleration(pp_rightSpeed,m_tsArduinoTime);
        TransmitSteer(pp_rightAngle,m_tsArduinoTime);
    }
    else if (DistanceTraveled <( pp_straightDuration + pp_forwardLeftDuration+ pp_leftDuration + pp_rightDuration) )
    {
        //TransmitHazardIndicator(tTrue,m_tsArduinoTime);
        //TransmitReverseIndicator(tTrue,m_tsArduinoTime);
        TransmitAcceleration(pp_leftSpeed,m_tsArduinoTime);
        TransmitSteer(pp_leftAngle,m_tsArduinoTime);
    }
    else if (DistanceTraveled < ( pp_straightDuration + pp_forwardLeftDuration + pp_leftDuration + pp_rightDuration + pp_backDuration))
    {
        //TransmitHazardIndicator(tTrue,m_tsArduinoTime);
        //TransmitReverseIndicator(tTrue,m_tsArduinoTime);
        TransmitAcceleration(pp_backSpeed,m_tsArduinoTime);
        TransmitSteer(pp_backAngle,m_tsArduinoTime);
    }
    else if (DistanceTraveled < ( pp_straightDuration + pp_forwardLeftDuration  + pp_leftDuration + pp_rightDuration + pp_backDuration + pp_ForwardLastDuration))
    {
        //TransmitHazardIndicator(tTrue,m_tsArduinoTime);
        //TransmitStartLaneTracking(tTrue,m_tsArduinoTime);
        //TransmitReverseIndicator(tTrue,m_tsArduinoTime);
        TransmitAcceleration(pp_ForwardLastSpeed,m_tsArduinoTime);
        TransmitSteer(pp_ForwardLastAngle,m_tsArduinoTime);
    }
    else 
    {
        ActionCompleted(1);
    }
}

void cDaedalusDecision::RunPullOverRightScript()
{
    /************ PullOver Right Properties *******************/
    TransmitRightIndicator(tTrue,m_tsArduinoTime);
    // if(m_bDebugModeEnabled)
    // {
    //     LOG_ERROR(cString::Format("RunPullOverRightScript: Started"));
    // }
    if (DistanceTraveled <  pr_straightDuration )
    {
        
        TransmitAcceleration(pr_straightSpeed,m_tsArduinoTime);
        TransmitSteer(pr_straightAngle,m_tsArduinoTime);
    } 
    else if (DistanceTraveled < ( pr_straightDuration + pr_rightDuration) )
    {
        TransmitAcceleration(pr_rightSpeed,m_tsArduinoTime);
        TransmitSteer(pr_rightAngle,m_tsArduinoTime);
         if(startLaneTracking)
        {
            TransmitStartLaneTracking(tTrue,m_tsArduinoTime);
            startLaneTracking = false;
        }
    }
    else 
    {
        ActionCompleted(1);
    }
}

void cDaedalusDecision::RunPullOverLeftScript()
{
    /************ PullOver Left Properties *******************/
    TransmitLeftIndicator(tTrue,m_tsArduinoTime);
    // if(m_bDebugModeEnabled)
    // {
    //     LOG_ERROR(cString::Format("RunPullOverLeftScript: Started"));
    // }
    if (DistanceTraveled <  pl_straightDuration )
    {
        TransmitAcceleration(pl_straightSpeed,m_tsArduinoTime);
        TransmitSteer(pl_straightAngle,m_tsArduinoTime);
    } 
    else if (DistanceTraveled < ( pl_straightDuration + pl_leftDuration) )
    {
        TransmitAcceleration(pl_leftSpeed,m_tsArduinoTime);
        TransmitSteer(pl_leftAngle,m_tsArduinoTime);
    }
    else 
    {
        ActionCompleted(1);
        
    }
}

void cDaedalusDecision::RunPullOverRightFromParallelParkingScript(){
    TransmitLeftIndicator(tTrue,m_tsArduinoTime);
    // if(m_bDebugModeEnabled)
    // {
    //     LOG_ERROR(cString::Format("RunPullOverRightFromParallelParkingScript: Started"));
    // }

    if (DistanceTraveled <  pr_pp_backDuration )
    {
        TransmitAcceleration(pr_pp_backSpeed,m_tsArduinoTime);
        TransmitSteer(pr_pp_backAngle,m_tsArduinoTime);
    } 
    else if (DistanceTraveled < ( pr_pp_backDuration + pr_pp_leftDuration) )
    {
        TransmitAcceleration(pr_pp_leftSpeed,m_tsArduinoTime);
        TransmitSteer(pr_pp_leftAngle,m_tsArduinoTime);

    }
    else if (DistanceTraveled < ( pr_pp_backDuration + pr_pp_leftDuration + pr_pp_forwardDuration) )
    {
        TransmitAcceleration(pr_pp_forwardSpeed,m_tsArduinoTime);
        TransmitSteer(pr_pp_forwardAngle,m_tsArduinoTime);
    }
    else if (DistanceTraveled < ( pr_pp_backDuration + pr_pp_leftDuration + pr_pp_forwardDuration + pr_pp_rightDuration) )
    {
        TransmitAcceleration(pr_pp_rightSpeed,m_tsArduinoTime);
        TransmitSteer(pr_pp_rightAngle,m_tsArduinoTime);
    }
    else 
    {
        ActionCompleted(1);
        
    }
}

void cDaedalusDecision::RunPullOverLeftFromParallelParkingScript()
{
    /************ PullOver Right Properties *******************/
    TransmitLeftIndicator(tTrue,m_tsArduinoTime);
    // if(m_bDebugModeEnabled)
    // {
    //     LOG_ERROR(cString::Format("RunPullOverLeftFromParallelParkingScript: Started"));
    // }
    if (DistanceTraveled <  pl_pp_backDuration )
    {
        TransmitAcceleration(pl_pp_backSpeed,m_tsArduinoTime);
        TransmitSteer(pl_pp_backAngle,m_tsArduinoTime);
    } 
    else if (DistanceTraveled < ( pl_pp_backDuration + pl_pp_forwardDuration) )
    {
        TransmitAcceleration(pl_pp_forwardSpeed,m_tsArduinoTime);
        TransmitSteer(pl_pp_forwardAngle,m_tsArduinoTime);
    }
    else if (DistanceTraveled < ( pl_pp_backDuration + pl_pp_leftDuration + pl_pp_forwardDuration) )
    {
        TransmitAcceleration(pl_pp_leftSpeed,m_tsArduinoTime);
        TransmitSteer(pl_pp_leftAngle,m_tsArduinoTime);
    }
    else 
    {
        ActionCompleted(1);
    }
}


/********************* Transmit Methods *****************************/

tResult cDaedalusDecision::TransmitFeedbackToDriverModule()
{
    if(!startDriving) RETURN_NOERROR;
    __synchronized_obj(m_oTransmitFeedbackToDriverCritSection);
    
    //create new media sample
    cObjectPtr<IMediaSample> pNewMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionDecisionOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));
    tBool bIsExecuted = true;
    tFloat32 timestamp = 0;
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionDecisionOutput, pNewMediaSample, pCoderOutput);    
 
        pCoderOutput->Set("ui32Timestamp", (tVoid*)&timestamp);
        pCoderOutput->Set("i32Value", (tVoid*)&DriverNextAction);
        pCoderOutput->Set("bIsExecuted", (tVoid*)&bIsExecuted);    
    }
              
    pNewMediaSample->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oDriverModuleFeedback.Transmit(pNewMediaSample));
    RETURN_NOERROR;
}

tResult cDaedalusDecision::FollowLane(int slowdown)
{

    switch (slowdown)
    {
        case 0: 
            TransmitAcceleration(LastLaneAceleration, InputTimeStamp);
            TransmitSteer(LastLaneSteerPT1, InputTimeStamp);
            break;
        case 1: //marker spotted
            TransmitAcceleration(LastLaneAceleration*GetPropertyFloat("Speed::SpeedDed_Threshold1"), InputTimeStamp);
            TransmitSteer(LastLaneSteerPT1, InputTimeStamp);
            break;
        case 2: //Walk to Crossroads
            TransmitAcceleration(GetPropertyFloat("Speed::SpeedDed_Threshold2"), InputTimeStamp);
            TransmitSteer(LastLaneSteerPT1, InputTimeStamp);
            break;
        case 3: //Slow After Actions
            TransmitAcceleration(LastLaneAceleration*GetPropertyFloat("Speed::SpeedDed_Threshold3"), InputTimeStamp);
            TransmitSteer(LastLaneSteerPT1, InputTimeStamp);
            break;
        case 4: //Pedestrian
            TransmitAcceleration(LastLaneAceleration*GetPropertyFloat("Speed::SpeedDed_Threshold4"), InputTimeStamp);
            TransmitSteer(LastLaneSteerPT1, InputTimeStamp);
            break;
        case 5: //WrongLane
            // LOG_ERROR(cString::Format("Slow Lane Speed : %f",GetPropertyFloat("Speed::SpeedDed_Threshold5")));
            TransmitAcceleration(GetPropertyFloat("Speed::SpeedDed_Threshold5"), InputTimeStamp);
            TransmitSteer(LastLaneSteerPT1, InputTimeStamp);
            break;
        case 6: //parking detection
            // LOG_ERROR(cString::Format("Slow Lane Speed : %f",GetPropertyFloat("Speed::SpeedDed_Threshold5")));
            TransmitAcceleration(GetPropertyFloat("Speed::SpeedDed_Threshold6"), InputTimeStamp);
            TransmitSteer(LastLaneSteerPT1, InputTimeStamp);
            break;
        case 7: //high speed
            // LOG_ERROR(cString::Format("Slow Lane Speed : %f",GetPropertyFloat("Speed::SpeedDed_Threshold5")));
            TransmitAcceleration(LastLaneAceleration*GetPropertyFloat("Speed::SpeedDed_Threshold7"), InputTimeStamp);
            TransmitSteer(LastLaneSteerPT1, InputTimeStamp);
            break;
        default: 
            TransmitAcceleration(LastLaneAceleration, InputTimeStamp);
            TransmitSteer(LastLaneSteerPT1, InputTimeStamp);
            break;
    }
    RETURN_NOERROR;
}

tResult cDaedalusDecision::TransmitStartLaneTracking(const tBool value, tTimeStamp tsInputTime)
{
    if(!startDriving) RETURN_NOERROR;
    __synchronized_obj(m_oTransmitStartLaneCritSection);
    
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
 
          pCoderOutput->Set("bValue", (tVoid*)&value);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
              
    pNewMediaSample->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(EnableLaneTrackingPin.Transmit(pNewMediaSample));
    RETURN_NOERROR;    
}

tResult cDaedalusDecision::TransmitSteer(const tFloat32 Steer, tTimeStamp tsInputTime)
{
    if(!startDriving) RETURN_NOERROR;
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
 
          pCoderOutput->Set("f32Value", (tVoid*)&Steer);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
              
    pNewMediaSample->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oSteerOutputPin.Transmit(pNewMediaSample));
    RETURN_NOERROR;
}

tResult cDaedalusDecision::TransmitAcceleration(tFloat32 Acceleration, tTimeStamp tsInputTime)
{
    if(!startDriving) RETURN_NOERROR;
    
    if(Acceleration == 0.0f)
        TransmitBrakeLights(true, InputTimeStamp);
    else
        TransmitBrakeLights(false, InputTimeStamp);
    __synchronized_obj(m_oTransmitAccelCritSection);
    
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
 
          pCoderOutput->Set("f32Value", (tVoid*)&Acceleration);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
              
    pNewMediaSample->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oAccelerateOutputPin.Transmit(pNewMediaSample));
    RETURN_NOERROR;
}

tResult cDaedalusDecision::TransmitRightIndicator(const tBool bTurnRightIndicator, tTimeStamp tsInputTime)
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

tResult cDaedalusDecision::TransmitLeftIndicator(const tBool bTurnLeftIndicator, tTimeStamp tsInputTime)
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

tResult cDaedalusDecision::TransmitHazardIndicator(const tBool value, tTimeStamp tsInputTime)
{
    __synchronized_obj(m_oTransmitHazardIndicatorCritSection);
    
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
 
          pCoderOutput->Set("bValue", (tVoid*)&value);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
              
    pNewMediaSample->SetTime(tsInputTime);
    RETURN_IF_FAILED(m_oHazardIndicatorPin.Transmit(pNewMediaSample));

    RETURN_NOERROR;
}

tResult cDaedalusDecision::TransmitReverseIndicator(const tBool bTurnOnIndicator, tTimeStamp tsInputTime)
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

tResult cDaedalusDecision::TransmitHeadlight(const tBool bHeadLights, tTimeStamp tsInputTime)
{
    if(!startDriving) RETURN_NOERROR;
    __synchronized_obj(m_oTransmitLightCritSection);
    
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
 
          pCoderOutput->Set("bValue", (tVoid*)&bHeadLights);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
              
    pNewMediaSample->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oHeadlightOutputPin.Transmit(pNewMediaSample));
    
        
    RETURN_NOERROR;    
}

tResult cDaedalusDecision::TransmitBrakeLights(const tBool bHeadLights, tTimeStamp tsInputTime)
{
    if(!startDriving) RETURN_NOERROR;
    __synchronized_obj(m_oTransmitBrakeLightCritSection);
    
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
 
          pCoderOutput->Set("bValue", (tVoid*)&bHeadLights);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
              
    pNewMediaSample->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oBrakeLightOutputPin.Transmit(pNewMediaSample));    
    RETURN_NOERROR;    
}


tResult cDaedalusDecision::TransmitDetectParking()
{
    if(!startDriving) RETURN_NOERROR;
    __synchronized_obj(m_oTransmitStartParkingDetCritSection); //
    
    //create new media sample
    cObjectPtr<IMediaSample> pNewMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescEnableParkingDetectionFilter->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));
    bool Val = true;
    {
          __adtf_sample_write_lock_mediadescription(m_pDescEnableParkingDetectionFilter, pNewMediaSample, pCoderOutput);    
 
          pCoderOutput->Set("bValue", (tVoid*)&Val);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }        
    pNewMediaSample->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oEnableParkingDetectionFilter.Transmit(pNewMediaSample));
    RETURN_NOERROR;
}

tResult cDaedalusDecision::TransmitDetectCrossroad(bool Val )
{
    if(!startDriving) RETURN_NOERROR;
    __synchronized_obj(m_oTransmitStartCrossroadDetCritSection); //
    
    //create new media sample
    cObjectPtr<IMediaSample> pNewMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescCrossroadDetectionInputFilter->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));
    {
          __adtf_sample_write_lock_mediadescription(m_pDescCrossroadDetectionInputFilter, pNewMediaSample, pCoderOutput);    
 
          pCoderOutput->Set("bValue", (tVoid*)&Val);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }        
    pNewMediaSample->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oEnableCrossroadDetectionFilter.Transmit(pNewMediaSample));
    RETURN_NOERROR;
}