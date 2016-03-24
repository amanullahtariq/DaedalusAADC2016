tResult cDaedalusDecision::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{

    if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        InputTimeStamp = pMediaSample->GetTime();
        m_bDebugModeEnabled=GetPropertyBool("m_bDebugModeEnabled");
         //Video input -- added because of action filter merging
        if(pSource == &m_iVideoInputPin)
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
                // if marker area in range (>1300) and x-axis translation is positive
                RVec[1] = (RVec[1] < 0)?RVec[1]+(2*3.14):RVec[1];
                
                if(f32Area > GetPropertyFloat("MarkerAreaThreshhold") && TVec[0] < 0 && RVec[1] >= GetPropertyFloat("MarkerRotationThreshhold"))
                {
                    if (markerId == MARKER_ID_NOMATCH)
                    {
                        Flag_NOMATCH=tTrue;
                        if (m_bDebugModeEnabled)   LOG_ERROR(cString::Format("DecisionFilter: Marker Id: Nothing"));                
                    }
                    else if (markerId == MARKER_ID_UNMARKEDINTERSECTION && !Flag_UNMARKEDINTERSECTION)
                    {
                        if (m_bDebugModeEnabled)   
                            LOG_ERROR(cString::Format("DecisionFilter: Marker Input: Id -> %d Area: %f, R: [%f , %f , %f]", markerId, f32Area, RVec[0],RVec[1],RVec[2]));
                        // cross road marker
                        int diff = InputTimeStamp - LastDetTime_UNMARKEDINTERSECTION;
                        if (diff < GetPropertyInt("MarkerTime_Threshold") || LastDetTime_UNMARKEDINTERSECTION==0)
                        {
                            markerInSight = GetPropertyInt("SLowSpeedForMarker") );
                            Count_UNMARKEDINTERSECTION++;
                            LastDetTime_UNMARKEDINTERSECTION=InputTimeStamp;
                        }else
                        {
                            LastDetTime_UNMARKEDINTERSECTION = 0;
                            Count_UNMARKEDINTERSECTION = 0;
                        }
                        if (Count_UNMARKEDINTERSECTION >= GetPropertyInt("CountConfirmDetection") )
                        {
                            ChangeFlow(2);
                            markerInSight = 0;
                            Flag_NOMATCH=tFalse;   
                            Count_UNMARKEDINTERSECTION = 0;
                            Flag_UNMARKEDINTERSECTION=tTrue;
                            DistanceToFollow = TVec[2] - GetPropertyFloat("DistanceCrossroadToMarker");
                            if ( DriverNextAction == Driver_right )
                            {
                                DistanceToFollow += GetPropertyFloat("RightTurnExtraDistance");
                            }  
                             if (m_bDebugModeEnabled)  LOG_ERROR(cString::Format("Marker Detected!! Following Lane for Dist : %f",DistanceToFollow));
                            // enable travel till crossroad Flag
                        }
                    }
                    else if (markerId == MARKER_ID_STOPANDGIVEWAY && !Flag_STOPANDGIVEWAY)
                    {
                        if (m_bDebugModeEnabled)   
                            LOG_ERROR(cString::Format("DecisionFilter: Marker Input: Id -> %d Area: %f, R: [%f , %f , %f]", markerId, f32Area, RVec[0],RVec[1],RVec[2]));
                        int diff = InputTimeStamp - LastDetTime_STOPANDGIVEWAY;
                        if (diff < GetPropertyInt("MarkerTime_Threshold") || LastDetTime_STOPANDGIVEWAY==0)
                        {
                            markerInSight = GetPropertyInt("SLowSpeedForMarker") );
                            Count_STOPANDGIVEWAY++;
                            LastDetTime_STOPANDGIVEWAY = InputTimeStamp;
                        }else
                        {
                            LastDetTime_STOPANDGIVEWAY = 0;
                            Count_STOPANDGIVEWAY = 0;
                        }
                        
                        if (Count_STOPANDGIVEWAY >= GetPropertyInt("CountConfirmDetection") )
                        {
                            ChangeFlow(2);
                            markerInSight = 0;
                            Flag_NOMATCH=tFalse;   
                            Count_STOPANDGIVEWAY = 0;
                            Flag_STOPANDGIVEWAY=tTrue;
                            DistanceToFollow = TVec[2] - GetPropertyFloat("DistanceCrossroadToMarker");
                            if ( DriverNextAction == Driver_right )
                            {
                                DistanceToFollow += GetPropertyFloat("RightTurnExtraDistance");
                            }                             
                             if (m_bDebugModeEnabled)  LOG_ERROR(cString::Format("Marker Detected!! Following Lane for Dist : %f",DistanceToFollow));
                            // enable travel till crossroad Flag
                        }
                    }
                    else if (markerId == MARKER_ID_HAVEWAY && !Flag_HAVEWAY)
                    {
                        if (m_bDebugModeEnabled)   
                            LOG_ERROR(cString::Format("DecisionFilter: Marker Input: Id -> %d Area: %f, R: [%f , %f , %f]", markerId, f32Area, RVec[0],RVec[1],RVec[2]));
                        int diff = InputTimeStamp - LastDetTime_HAVEWAY;
                        if (diff < GetPropertyInt("MarkerTime_Threshold") || LastDetTime_HAVEWAY==0)
                        {
                            markerInSight = GetPropertyInt("SLowSpeedForMarker") );;
                            Count_HAVEWAY++;
                            LastDetTime_HAVEWAY = InputTimeStamp;
                        }else
                        {
                            LastDetTime_HAVEWAY = 0;
                            Count_HAVEWAY = 0;
                        }
                        
                        if (Count_HAVEWAY >= GetPropertyInt("CountConfirmDetection") )
                        {
                            ChangeFlow(2);
                            markerInSight = 0;
                            Flag_NOMATCH=tFalse;   
                            Count_HAVEWAY = 0;
                            TVec_DistanceToMarker = TVec[2];
                            Flag_HAVEWAY=tTrue;
                            DistanceToFollow = TVec_DistanceToMarker - GetPropertyFloat("DistanceCrossroadToMarker");
                            if ( DriverNextAction == Driver_right )
                            {
                                DistanceToFollow += GetPropertyFloat("RightTurnExtraDistance");
                            } 
                             if (m_bDebugModeEnabled)  LOG_ERROR(cString::Format("Marker Detected!! Following Lane for Dist : %f",DistanceToFollow));
                            // enable travel till crossroad Flag
                        }
                    }
                    else if (markerId == MARKER_ID_GIVEWAY && !Flag_GIVEWAY)
                    {
                        if (m_bDebugModeEnabled)   
                            LOG_ERROR(cString::Format("DecisionFilter: Marker Input: Id -> %d Area: %f, R: [%f , %f , %f]", markerId, f32Area, RVec[0],RVec[1],RVec[2]));
                        int diff = InputTimeStamp - LastDetTime_GIVEWAY;
                        if (diff < GetPropertyInt("MarkerTime_Threshold") || LastDetTime_GIVEWAY==0)
                        {
                            markerInSight = GetPropertyInt("SLowSpeedForMarker") );;
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
                            markerInSight = 0;
                            Flag_NOMATCH=tFalse;   
                            Count_GIVEWAY = 0;
                            TVec_DistanceToMarker = TVec[2];
                            Flag_GIVEWAY=tTrue;
                            DistanceToFollow = TVec_DistanceToMarker - GetPropertyFloat("DistanceCrossroadToMarker");
                            if ( DriverNextAction == Driver_right )
                            {
                                DistanceToFollow += GetPropertyFloat("RightTurnExtraDistance");
                            } 
                             if (m_bDebugModeEnabled)  LOG_ERROR(cString::Format("Marker Detected!! Following Lane for Dist : %f",DistanceToFollow));
                            // enable travel till crossroad Flag
                        }
                    }
                    else if (markerId == MARKER_ID_PARKINGAREA && !Flag_ParkingDetection  && !Flag_PARKINGAREA)
                    {
                        if (m_bDebugModeEnabled)   
                            LOG_ERROR(cString::Format("DecisionFilter: Marker Input: Id -> %d Area: %f, R: [%f , %f , %f]", markerId, f32Area, RVec[0],RVec[1],RVec[2]));
                        if(DriverNextAction != Driver_parallel_parking && DriverNextAction != Driver_cross_parking)
                            RETURN_NOERROR;
                        if( f32Area < 750)
                            RETURN_NOERROR;
                        int diff = InputTimeStamp - LastDetTime_PARKINGAREA;
                        if (diff < GetPropertyInt("MarkerTime_Threshold") || LastDetTime_PARKINGAREA==0)
                        {
                            iter = GetPropertyInt("slowIterationsAfterTurn");
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
                if(startDriving && execFlow == 1) //TODO: Create var 1 == lanetracking
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
                Reinitiaze();
            }
            else
                startDriving = true;
            if (m_bDebugModeEnabled)   
                LOG_ERROR(cString::Format("DecisionFilter: DriverModule Data: ID -> %d time: %d isProcessed: %d",DriverNextAction, DriverActionTime,DriverIsProcessed));
                
        }
        else if(pSource == &m_iWheelEncoderDistance)
        {
            WheelEncoderDistance = 0.0;  //TODO: Create Global
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

            if(startDriving && execFlow == 2) //TODO: Create var 2 == wheelEncoder
            {
                MakeDecision();    
            }   
        }
        else if(pSource==&m_iObjectFilter)
        {
            LOG_ERROR("stop everything:There is an object");
        }
        else if(pSource==&m_iSensorFilter)
        {
        
            tFloat32 us_front = 0.0f;
            tFloat32 us_front_left = 0.0f;
            
            
            {   
                __adtf_sample_read_lock_mediadescription(m_pDescriptionOutputSensor,pMediaSample,pCoderInput);       
                // get IDs
                if(!ObstacleSensorModuleBufferReaderFlag)
                {
                    pCoderInput->GetID("minimum_dist[0]", SensorInputFrontBuffer);
                    pCoderInput->GetID("minimum_dist[1]", SensorInputFrontLeftBuffer);
                    pCoderInput->GetID("minimum_dist[2]", SensorInputFrontRightBuffer);
                    ObstacleSensorModuleBufferReaderFlag = tTrue;
                }  
                // get the values from Action Module
                pCoderInput->Get(SensorInputFrontBuffer, (tVoid*)&us_front);
                pCoderInput->Get(SensorInputFrontLeftBuffer, (tVoid*)&us_front_left);   
                pCoderInput->Get(SensorInputFrontRightBuffer, (tVoid*)&us_front_right);  
            }
             //TODO: Flow based threshholding for US sensor data
            if (us_front != 0 || us_front_left!=0 || us_front_right!=0)
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
                    pCoderInput->GetID("ui32ArduinoTimestamp",WheelEncoderDistanceTimeBuffer);
                    ParkingDetectionModuleBufferReaderFlag = tTrue;
                }  
                // get the values from Action Module
                pCoderInput->Get(DistanceToParkingBuffer, (tVoid*)&DistanceToParking );
             }
             if(DistanceToParking > 0.0f){
                
            if (m_bDebugModeEnabled)   
                LOG_ERROR(cString::Format("Decision: Dist: D -> %f",DistanceToParking));
                
                DistanceToParking = (DistanceToParking * GetPropertyFloat("ParkDistScale")) + GetPropertyFloat("ParkDistTransform"); 
                Flag_CoverDistanceBeforeTurn = true;
                Flag_ParkingDetection = false;
                Flag_PARKINGAREA = true;
             }
             // do the logic
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
                }else if(pSource == &m_iActionFilterFeedback)
                {
                    m_pDescriptionActionFeedbackOutput = pMediaTypeDesc;
                }else if(pSource == &m_iObjectFilter)
                {
                    m_pDescriptionOutputObject = pMediaTypeDesc;
                }else if(pSource == &m_iSensorFilter){
                    m_pDescriptionOutputSensor = pMediaTypeDesc;
                }
                else if(pSource == &m_oParkingDetectionInput){
                    m_pDescParkingDetectionFilter = pMediaTypeDesc;
                }
        }        
    }
    RETURN_NOERROR;
}

tResult cDaedalusDecision::MakeDecision()
{
    if(FlagObs == true)
    {
        TransmitAcceleration(0.0,InputTimeStamp)
    }
    else(execFlow == 1) //Lanetracking
    { 
        if(markerInSight > 0) //TODO: New Property
        {
            markerInSight--; 
            FollowLane(1)
        }else
            FollowLane(0);
            
    }else(execFlow == 2) // wheelEncoder
    {
        if(EncoderState == 1){
            DistanceTraveled = DistanceTraveled + WheelEncoderDistance;
            float Diff = DistanceToCrossRoad - DistanceTraveled;
            if (Diff < GetPropertyFloat("DistanceErrorThreshhold"))
            {
                if(m_bDebugModeEnabled)
                    LOG_ERROR(cString::Format("Car Turning Detected, Diff to Groud Truth %f",Diff));
                EncoderState = 2;
                iter = 0;
                DistanceTraveled = 0.0;
            }
            else
            {
                if(Flag_HAVEWAY && DriverNextAction == Driver_straight){
                    FollowLane(0); //dont slow
                }else{
                    FollowLane(2);  //do slow
                }
            }
        }else if (EncoderState == 2) // Pre Action
        {
            if(Flag_STOPANDGIVEWAY == tTrue || Flag_GIVEWAY == tTrue)
            {
                if(iter < GetPropertyInt("StoppingIterations")){
                    iter ++;
                    TransmitAcceleration(0.0f, InputTimeStamp);
                }else {
                    EncoderState = 3;
                    TransmitStartLaneTracking(tFalse,InputTimeStamp);
                }
                
            }else{
                EncoderState = 3;
                DistanceTraveled = 0.0;
            }
        }else if(EncoderState == 3){
                RunAction();
                DistanceTraveled = DistanceTraveled + WheelEncoderDistance;
        }
    }    

    RETURN_NOERROR;
}


void cDaedalusDecision::Reinitiaze(){ //TODO : FIX Variables here
    Initialize();
    TransmitAcceleration(0.0f,InputTimeStamp);
    TransmitStartLaneTracking(true,InputTimeStamp);
    startDriving = false;
    InteractiveLaneFollowing = false;
    us_front_right = 0.0f;
    Flag_UNMARKEDINTERSECTION=tFalse;
    Flag_STOPANDGIVEWAY=tFalse;
    Flag_PARKINGAREA=tFalse;
    Flag_HAVEWAY=tFalse;
    Flag_GIVEWAY=tFalse;
    Flag_ONEWAYSTREET=tFalse;
    Flag_NOMATCH=tTrue;
    Count_UNMARKEDINTERSECTION = 0;
    Count_STOPANDGIVEWAY=0;
    Count_PARKINGAREA=0;
    Count_HAVEWAY=0;
    Count_GIVEWAY=0;
    //Last Detected time variable for all markers
    LastDetTime_UNMARKEDINTERSECTION = 0;
    LastDetTime_STOPANDGIVEWAY=0;
    LastDetTime_PARKINGAREA=0;
    LastDetTime_HAVEWAY=0;
    LastDetTime_GIVEWAY=0;

    Flag_CoverDistanceBeforeTurn = false;
    FlagObs = false;
    Flag_ParkingDetection = false;
    DistanceToFollow = 0.0f;
    DistanceTraveled = 0.0f;
    DistanceToParking = 0.0f;
    DriverNextAction = -1;
    StopWheelEncoder = false;
    Flag_StopLaneTracking = false;
    iter=0;
}

 void cDaedalusDecision::ChangeFlow(int a)
{
    execFlow == a;
    DistanceTraveled = 0; 
}

void cDaedalusDecision::ActionCompleted(){
    prevDriverAction = DriverNextAction;
    DriverNextAction = -1;
    TransmitFeedbackToDriverModule();
    TransmitStartLaneTracking(tTrue,m_tsArduinoTime);
    ChangeFlow(1);
    TransmitAcceleration(0.0f,m_tsArduinoTime);
    TransmitSteer(90.0,m_tsArduinoTime);
    TransmitHazardIndicator(tFalse,m_tsArduinoTime);
    TransmitReverseIndicator(tFalse,m_tsArduinoTime);
    TransmitRightIndicator(tFalse,m_tsArduinoTime);
    TransmitLeftIndicator(tFalse,m_tsArduinoTime);
    TransmitStartLaneTracking(tTrue,m_tsArduinoTime);
}