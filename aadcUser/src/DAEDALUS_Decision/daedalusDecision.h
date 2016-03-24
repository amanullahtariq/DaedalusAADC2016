#ifndef _DECISION_FILTER_H_
#define _DECISION_FILTER_H_

#include "ParkingType.h"
#include "../utile/object/object.h"
#define OID_ADTF_DAEDALUS_DECISION_FILTER "adtf.daedalus.decision"
#define CountConfirmDetection 5
#define MarkerAreaThreshhold 700.0
#define DistanceCrossroadToMarker 1.65 
#define DistanceErrorThreshhold 0.08
#define MarkerTime_Threshold 2000 // 20 centemeter

//Property for marker area threshhold
//********************************************************************************************************
class cDaedalusDecision : public adtf::cFilter            
{
    ADTF_FILTER(OID_ADTF_DAEDALUS_DECISION_FILTER, "DAEDALUS decision", adtf::OBJCAT_DataFilter);

protected:
    cVideoPin m_iVideoInputPin;
    cInputPin   m_iLaneTrackingStructInput;         // lane tracking input pins 
    cInputPin   m_iPinRoadSignExt;                  // Wheel Encoder Detection Input
    cInputPin   m_iWheelEncoderDistance;            // Wheel Encoder Detection Input
    cInputPin   m_iDriverAction;                    // Driver Module Inputs    
    cInputPin   m_iObjectFilter;                    // Input pin for object filter 
    cInputPin   m_iSensorFilter;                    // Input pin for object filter 
    cInputPin   m_oParkingDetectionInput;
    cInputPin   m_oCrossroadDetectionInput;
    //output pins
    cOutputPin  m_oAccelerateOutputPin;                  /*! Output Pin for accelerate values; values must be between 0...180*/
    cOutputPin  m_oReverseLightOutputPin;                /*! Output Pin for reverse light messages */
    cOutputPin  m_oHeadlightOutputPin;                    /*! Output Pin for head light */
    cOutputPin  m_oTurnSignalLeftOutputPin;                /*! Output Pin for indicator turn signal left  */
    cOutputPin  m_oTurnSignalRightOutputPin;                /*! Output Pin for indicator turn singal right  */
    cOutputPin  m_oHazardIndicatorPin;                         /*! Output pin for the hazzard light*/
    cOutputPin  m_oBrakeLightOutputPin;                    /*! Output Pin for brake light */
    cOutputPin  m_oSteerOutputPin;                       /*! Output Pin for steer angle messages; values must be between 0...180 */
    cOutputPin  m_oReverseIndicatorOutputPin;
    // out to other filters
    cOutputPin  m_oDriverModuleFeedback;                    /*! Output pin for Driver Filter */
    cOutputPin EnableLaneTrackingPin;
    cOutputPin  m_oEnableParkingDetectionFilter;
    cOutputPin  m_oEnableCrossroadDetectionFilter;      
public:
    cDaedalusDecision(const tChar* __info);
    virtual ~cDaedalusDecision();

private:

    /*! creates all the input Pins*/
    tResult CreateInputPins(__exception = NULL);
    /*! creates all the output Pins*/
    tResult CreateOutputPins(__exception = NULL);

    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescBool;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescLaneTracking;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionRoadSignExt;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionDecisionOutput;
    //Media type for reading from input pins
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputMarker;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputLane;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionDecisionInput;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputSampleDistance;
    cObjectPtr<IMediaTypeDescription> m_pDescParkingDetectionFilter, m_pDescEnableParkingDetectionFilter,m_pDescCrossroadDetectionFilter,m_pDescCrossroadDetectionInputFilter;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputObject , m_pDescriptionOutputSensor;
    //marker buffers
    tBufferID m_szIDRoadSignI16Identifier; 
    tBufferID m_szIDRoadSignF32Imagesize;  
    tBufferID m_szIDRoadSignF32Translation0;
    tBufferID m_szIDRoadSignF32Translation1;
    tBufferID m_szIDRoadSignF32Translation2;
    tBufferID m_szIDRoadSignF32Rotation0;
    tBufferID m_szIDRoadSignF32Rotation1;
    tBufferID m_szIDRoadSignF32Rotation2;
    // Lane Buffer
    tBufferID AccelerationBuffer; 
    tBufferID SteeringBuffer; 
    tBufferID SteeringPT1Buffer;
    tBufferID HeadlightBuffer; 
    tBufferID TimeStampBuffer;
    //driver buffer
    tBufferID DriverNextActionBuffer; 
    tBufferID DriverTimeBuffer; 
    tBufferID DriverIsProcessedBuffer; 
    //Wheel Encoder buffer
    tBufferID WheelEncoderDistanceBuffer;
    tBufferID WheelEncoderDistanceTimeBuffer;
    //Emergency Filter buffer
    tBufferID SensorInputFrontBuffer;
    tBufferID SensorInputFrontRightBuffer;
    tBufferID SensorInputFrontLeftBuffer,SensorInputRightBuffer;
    //Parking Detection
    tBufferID DistanceToParkingBuffer,CrossRoadDetBuffer;
    //reading booleans
    tBool m_bIDsRoadSignSet;
    tBool LaneDetectionBufferReaderFlag;
    tBool DriverModuleBufferReaderFlag;
    tBool WheelEncoderModuleBufferReaderFlag;
    tBool ObstacleSensorModuleBufferReaderFlag;
    tBool ParkingDetectionModuleBufferReaderFlag, CrossroadDetectionModuleBufferReaderFlag;
    //Critical section output support
    cCriticalSection    m_oTransmitSteerCritSection;
    cCriticalSection    m_oTransmitAccelCritSection;
    cCriticalSection    m_oTransmitLightCritSection;
    cCriticalSection    m_oTransmitFeedbackToDriverCritSection;
    cCriticalSection    m_oTransmitStartLaneCritSection;
    cCriticalSection    m_oTransmitStartParkingDetCritSection,m_oTransmitStartCrossroadDetCritSection;
    tUInt32     m_tsArduinoTime;
    
    //debugmode

    tFloat32 us_front;
    tFloat32 us_front_left;
    tFloat32 us_front_right;
    tFloat32 us_right;

    bool m_bDebugModeEnabled;
    std::vector<Object> ObjectsDetected;
    int execFlow,EncoderState,parkedBlinking;
    tFloat32 WheelEncoderDistance;
    int DriveSlowAfterTurn,markerInSight;
    // Lane Detection Previous Recordings 
    tFloat32 LastLaneAceleration;
    tFloat32 LastLaneSteer;
    tFloat32 LastLaneSteerPT1;
    tBool LastlaneHeadlight;
    tUInt32 LastLaneTimestamp;
    tTimeStamp InputTimeStamp,ObjectsDetectedTime;  
    
    //Drive Straight Variables
    tFloat32 LanePreviousVelocity;
    tUInt32 LanePreviousTimeStamp;
    tFloat32 LanePreviousAnglePTI; 
    tFloat32 DistanceTraveled;         
    tFloat32 DistanceToFollow;      // z-axis from translation matrix
    tFloat32 DistanceToParking;
    
    //Variable name for cross detetion check
    tTimeStamp LastDetTime_UNMARKEDINTERSECTION;
    tTimeStamp LastDetTime_STOPANDGIVEWAY;
    tTimeStamp LastDetTime_PARKINGAREA;
    tTimeStamp LastDetTime_HAVEWAY;
    tTimeStamp LastDetTime_AHEADONLY;
    tTimeStamp LastDetTime_GIVEWAY;
    tTimeStamp LastDetTime_PEDESTRIANCROSSING;
    tTimeStamp LastDetTime_NOOVERTAKING;
    tTimeStamp LastDetTime_MoveAroundObs;
    // marker flags
    tBool Flag_UNMARKEDINTERSECTION;
    tBool Flag_STOPANDGIVEWAY;
    tBool Flag_PARKINGAREA;
    tBool Flag_HAVEWAY;
    tBool Flag_AHEADONLY;
    tBool Flag_GIVEWAY;
    tBool Flag_NOOVERTAKING;
    tBool Flag_PEDESTRIANCROSSING;
    bool Flag_ParkingDetection;
    tBool Flag_NOMATCH;
    int Count_UNMARKEDINTERSECTION;
    int Count_STOPANDGIVEWAY;
    int Count_PARKINGAREA;
    int Count_HAVEWAY;
    int Count_AHEADONLY;
    int Count_GIVEWAY;
    int Count_PEDESTRIANCROSSING;
    int Count_NOOVERTAKING;
    int CountMoveAroundObs;
    int Count_BackToLane;
    
    int iter;
    tInt32 DriverNextAction;
    bool startDriving;
    bool FlagObs;
    int CrossRoadDetected;
    // transmition functions
    void ChangeFlow(int flow);
    void ActionCompleted(bool TellJury);
    void Reinitiaze();    
    void Stop();    
    tResult ProcessObstaclesOnSide();
    tResult ProcessObstaclesOnCrossroad();
    tResult ProcessObstaclesOnLane(bool detected);
    tResult TransmitAcceleration(tFloat32, tTimeStamp);
    tResult TransmitSteer(tFloat32, tTimeStamp);
    tResult TransmitBrakeLights(const tBool bHeadLights, tTimeStamp tsInputTime);
    tResult TransmitHeadlight(const tBool bHeadLights, tTimeStamp tsInputTime);
    tResult TransmitFeedbackToDriverModule();
    tResult TransmitStartLaneTracking(const tBool value, tTimeStamp tsInputTime);
    tResult TransmitDetectParking();
    tResult TransmitRightIndicator(const tBool, tTimeStamp);
    tResult TransmitLeftIndicator(const tBool, tTimeStamp);
    tResult TransmitReverseIndicator(const tBool, tTimeStamp);
    tResult TransmitHazardIndicator(const tBool, tTimeStamp);
    tResult TransmitDetectCrossroad(bool Val );

    
protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);
    tResult MakeDecision();
    tResult CoverDistanceBeforeTurn(tFloat32 WheelEncoderDistance);
    tResult FollowLane(int a);

    //ACtion filter properties
private: 
    int waitBeforeChangeLane;
    bool CurrLane;
    bool startCheckForParkingType;
    bool m_bShowLog;
    bool m_bFirstFrame;
    bool startLaneTracking;
    bool activeActionScript;
    tBitmapFormat m_sInputFormat;

    tUInt8 m_ui8InitCtrl;
    tInt32 prevDriverAction;
    tInt32 lastCommand;
    tInt32 cameraOffset;
    Mat inputImage;
    ParkingType parkingType;

    int pp_counter;
    int cp_counter;
    int total_counter;

    //Straight Properties
    tFloat32 st_straightAngle;
    tFloat32 st_straightSpeed;
    tFloat32 st_straightDuration;

    //Left Turn Properties
    tFloat32 lt_leftAngle;
    tFloat32 lt_leftSpeed;
    tFloat32 lt_leftDuration;
    tFloat32 lt_straightAngle;
    tFloat32 lt_straightSpeed;
    tFloat32 lt_straightDuration;
    tFloat32 lt_endDuration;
    tFloat32 lt_endSpeed;
    tFloat32 lt_endAngle;

    //Right Turn Properties
    tFloat32 rt_rightAngle;
    tFloat32 rt_rightSpeed;
    tFloat32 rt_rightDuration;
    tFloat32 rt_straightAngle;
    tFloat32 rt_straightSpeed;
    tFloat32 rt_straightDuration;

    //Cross Parking Properties
    tFloat32 cp_leftAngle;
    tFloat32 cp_straightAngle;
    tFloat32 cp_rightAngle;
    tFloat32 cp_backAngle;

    tFloat32 cp_leftSpeed;
    tFloat32 cp_straightSpeed;
    tFloat32 cp_rightSpeed;
    tFloat32 cp_backSpeed;

    tFloat32 cp_leftDuration;
    tFloat32 cp_straightDuration;
    tFloat32 cp_rightDuration;
    tFloat32 cp_backDuration;

    //Parallel Parking Properties
    tFloat32 pp_leftAngle;
    tFloat32 pp_forwardLeftAngle;
    tFloat32 pp_straightAngle;
    tFloat32 pp_rightAngle;
    tFloat32 pp_backAngle;
    tFloat32 pp_ForwardLastAngle;

    tFloat32 pp_leftSpeed;
    tFloat32 pp_forwardLeftSpeed;
    tFloat32 pp_straightSpeed;
    tFloat32 pp_rightSpeed;
    tFloat32 pp_backSpeed;
    tFloat32 pp_ForwardLastSpeed;

    tFloat32 pp_leftDuration;
    tFloat32 pp_forwardLeftDuration;
    tFloat32 pp_straightDuration;
    tFloat32 pp_rightDuration;
    tFloat32 pp_backDuration;
    tFloat32 pp_ForwardLastDuration;

    //Pull out left properties
    tFloat32 pl_leftSpeed;
    tFloat32 pl_leftAngle;
    tFloat32 pl_leftDuration;
    tFloat32 pl_straightAngle;
    tFloat32 pl_straightSpeed;
    tFloat32 pl_straightDuration;

    //Pull out right properties
    tFloat32 pr_rightSpeed;
    tFloat32 pr_rightAngle;
    tFloat32 pr_rightDuration;
    tFloat32 pr_straightAngle;
    tFloat32 pr_straightSpeed;
    tFloat32 pr_straightDuration;

    //Pull out right properties
    tFloat32 pr_pp_backSpeed;
    tFloat32 pr_pp_leftSpeed;
    tFloat32 pr_pp_rightSpeed;
    
    tFloat32 pr_pp_backAngle;
    tFloat32 pr_pp_leftAngle;
    tFloat32 pr_pp_rightAngle;

    tFloat32 pr_pp_backDuration;
    tFloat32 pr_pp_leftDuration;
    tFloat32 pr_pp_rightDuration;

    tFloat32 pr_pp_forwardSpeed;
    tFloat32 pr_pp_forwardAngle ;
    tFloat32 pr_pp_forwardDuration;

    //Pull out left Parallel properties
    tFloat32 pl_pp_backSpeed;
    tFloat32 pl_pp_forwardSpeed;
    tFloat32 pl_pp_leftSpeed;
    
    tFloat32 pl_pp_backAngle;
    tFloat32 pl_pp_forwardAngle;
    tFloat32 pl_pp_leftAngle;

    tFloat32 pl_pp_backDuration;
    tFloat32 pl_pp_forwardDuration;
    tFloat32 pl_pp_leftDuration;

    //change lane properties
    tFloat32 cl_backSpeed;
    tFloat32 cl_forwardSpeed;
    tFloat32 cl_leftSpeed;
    tFloat32 cl_rightSpeed;
    
    tFloat32 cl_backAngle;
    tFloat32 cl_forwardAngle;
    tFloat32 cl_leftAngle;
    tFloat32 cl_rightAngle;

    tFloat32 cl_backDuration;
    tFloat32 cl_forwardDuration;
    tFloat32 cl_leftDuration;
    tFloat32 cl_rightDuration;


    // Critical Sections
    cCriticalSection m_oTransmitRightIndicatorCritSection;
    cCriticalSection m_oTransmitLeftIndicatorCritSection;
    cCriticalSection m_oTransmitHazardIndicatorCritSection;
    cCriticalSection m_oTrasmitReverseIndicatorCritSection;
    cCriticalSection LightHandlerCriticalSection;
    cCriticalSection TransmitSignalCritSection;
    cCriticalSection m_oTransmitBrakeLightCritSection;


    tResult ProcessForFirstFrame();
    tResult ReadProperties();
    tResult ProcessInput(IMediaSample* pSample, tTimeStamp);
    tVoid InitializeProperties();
    
    /************ Set Properties Methods ******************/
    void SetAllProperties();
    void SetStraightProperties();
    void SetLeftTurnProperties();
    void SetRightTurnProperties();
    void SetCrossParkingProperties();
    void SetParallelParkingProperties();
    void SetPullOverLeftProperties();
    void SetPullOverRightProperties();
    void SetPullOverLeftFromParallelParkingProperties();
    void SetPullOverRightFromParallelParkingProperties();
    void SetChangeLaneProperties();

    /************ Get Properties Methods ******************/
    void GetStraightProperties();
    void GetLeftTurnProperties();
    void GetRightTurnProperties();
    void GetCrossParkingProperties();
    void GetParallelParkingProperties();
    void GetPullOverLeftProperties();
    void GetPullOverRightProperties();
    void GetPullOverLeftFromParallelParkingProperties();
    void GetPullOverRightFromParallelParkingProperties();
    void GetChangeLaneProperties();
    void CheckParkingTypeAndPullOutRight();

    /************* Scripts Methods ******************/
    void RunParallelParkingScript();
    void RunTurnLeftScript();
    void RunTurnRightScript();
    void RunStraightScript();
    void RunCrossParkingScript();
    void RunPullOverLeftScript();
    void RunPullOverRightScript();
    void RunPullOverLeftFromParallelParkingScript();
    void RunPullOverRightFromParallelParkingScript();
    void RunChangeLaneLeftScript();
    void RunChangeLaneRightScript();

    /************* Transmit Methods ******************/
    tResult RunAction(int Action);

};

//*************************************************************************************************
#endif // _DECISION_FILTER_H_
