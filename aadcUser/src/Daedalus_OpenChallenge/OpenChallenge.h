#ifndef _OpenChallenge_H_
#define _OpenChallenge_H_
#define OID_ADTF_OpenChallenge  "adtf.OpenChallenge"

 

class cOpenChallenge : public adtf::cTimeTriggeredFilter {

  ADTF_FILTER(OID_ADTF_OpenChallenge, "OpenChallenge", adtf::OBJCAT_DataFilter);
 protected:

  cInputPin   m_iSensorFilter; 

  cOutputPin  m_oAccelerateOutputPin; 
  cOutputPin  m_oSteerOutputPin;   
  cOutputPin  m_oReverseIndicatorOutputPin;
  cOutputPin  m_oHeadlightOutputPin;  
  cOutputPin  m_oTurnSignalLeftOutputPin;   
  cOutputPin  m_oTurnSignalRightOutputPin;
  cOutputPin  m_oBrakeLightOutputPin; 
  /*! output pin for the decision module */
  cOutputPin  m_DecisionOutputPin;

	
protected: // implement cTimeTriggeredFilter
  tResult Cycle(__exception = NULL);
  string prevLeapMotionCommand;
  bool firstTime;
  tUInt32 m_f32IntervalTime;
  tUInt32 m_tsArduinoTime;
  bool isAutoPilot;
  tTimeStamp InputTimeStamp;  
  int startCounter;
  bool showlog;
  bool FlagObs;
  int Port;
  int noCommandRecieved;
  //For TCP connection
  int clientSocket;
  struct sockaddr_in serverAddr;
  socklen_t addr_size;

  // For car running properties
  tFloat32 noSpeed;
  tFloat32 maxSpeed;
  tFloat32 medSpeed;
  tFloat32 minSpeed;

  tFloat32 maxLeftAngle;
  tFloat32 medLeftAngle;
  tFloat32 minLeftAngle;

  tFloat32 maxRightAngle;
  tFloat32 medRightAngle;
  tFloat32 minRightAngle;

  tFloat32 straightAngle;
private:
  cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
  cObjectPtr<IMediaTypeDescription> m_pCoderDescBool;
  cObjectPtr<IMediaTypeDescription> m_pDescriptionDecisionOutput;
  cObjectPtr<IMediaTypeDescription>  m_pDescriptionOutputSensor;
  
  cCriticalSection m_oTransmitLightCritSection;
  cCriticalSection m_oTransmitRightIndicatorCritSection;
  cCriticalSection m_oTransmitLeftIndicatorCritSection;
  cCriticalSection m_oTrasmitReverseIndicatorCritSection;
  cCriticalSection m_oTransmitBrakeLightCritSection;
  cCriticalSection m_oTransmitSteerCritSection;
  cCriticalSection m_oTransmitAccelCritSection;
  cCriticalSection m_oTransmitActionCritSection;
  

  tBool ObstacleSensorModuleBufferReaderFlag;


  //Emergency Filter buffer
  tBufferID SensorInputFrontBuffer;
  tBufferID SensorInputFrontRightBuffer;
  tBufferID SensorInputFrontLeftBuffer,SensorInputRightBuffer;
		
 public:
    cOpenChallenge(const tChar* __info);
    virtual ~cOpenChallenge();

protected:
    tResult Init(tInitStage eStage, __exception);
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);

  // Custom Methods
 private:
  tResult CreateInputPins(__exception = NULL);
  tResult CreateOutputPins(__exception = NULL);
  tResult TransmitAcceleration(tFloat32, tTimeStamp);
  tResult TransmitSteer(tFloat32, tTimeStamp);
  tResult TransmitBrakeLights(const tBool bHeadLights, tTimeStamp tsInputTime);
  tResult TransmitHeadlight(const tBool bHeadLights, tTimeStamp tsInputTime);
  tResult TransmitRightIndicator(const tBool, tTimeStamp);
  tResult TransmitLeftIndicator(const tBool, tTimeStamp);
  tResult TransmitReverseIndicator(const tBool, tTimeStamp);
  tResult TransmitHazardIndicator(const tBool, tTimeStamp);
  tResult TransmitAction(cString m_sCurrentManeuver);
  void TurnOffAllLights();
  void TurnOnLights();
  tVoid ProcessInputCommand(string LeapMotionCommand);
  void StopCar();
  void GetAllProperties();
  tResult GetMessageFromTCP();
  tVoid ConnectToTCP();
  bool StopOnObstacle(string LeapMotionCommand);
};

//*************************************************************************************************
#endif // _OpenChallenge_H_
