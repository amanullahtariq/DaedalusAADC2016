#ifndef _EMERGENCY_FILTER_H_
#define _EMERGENCY_FILTER_H_

#define OID_ADTF_DAEDALUS_EMERGENCY  "adtf.daedalus.emergency"
#include "../utile/object/object.h"

class cEmergency : public adtf::cTimeTriggeredFilter {

  ADTF_FILTER(OID_ADTF_DAEDALUS_EMERGENCY, "DAEDALUS_emergency", adtf::OBJCAT_DataFilter);
 protected:
  //Front Sensor
  cInputPin    m_usSensorFront;
  cInputPin    m_usSensorFrontLeft;
  cInputPin    m_usSensorFrontRight;
  cInputPin    m_usSensorFrontLeftCenter;
  cInputPin    m_usSensorFrontRightCenter;
  //side sensor
  cInputPin    m_usSensorSideLeft;
  cInputPin    m_usSensorSideRight;
  //Back sensor
  cInputPin    m_usSensorRearLeft;
  cInputPin    m_usSensorRearCenter;
  cInputPin    m_usSensorRearRight;

  // Output pin
  cOutputPin   m_sensor_obstacle_value;

  
 private:
  tResult CreateInputPins(__exception = NULL);
  tResult CreateOutputPins(__exception = NULL);
	tResult TransmitSensorObjectInfoToDecisionModule(tFloat32 F,tFloat32 FL, tFloat32 FR, tFloat32 L, tFloat32 R, tFloat32 B);
	tResult ProcessObjectSensor();
    
  protected: // implement cTimeTriggeredFilter
        tResult Cycle(__exception = NULL);
  cObjectPtr<IMediaTypeDescription> sensorMediaTypeDescriptor0;
  cObjectPtr<IMediaTypeDescription> sensorMediaTypeDescriptor1;
  cObjectPtr<IMediaTypeDescription> sensorMediaTypeDescriptor2;
  cObjectPtr<IMediaTypeDescription> sensorMediaTypeDescriptor3;
  cObjectPtr<IMediaTypeDescription> sensorMediaTypeDescriptor4;
  cObjectPtr<IMediaTypeDescription> sensorMediaTypeDescriptor5;
  cObjectPtr<IMediaTypeDescription> sensorMediaTypeDescriptor6;
  cObjectPtr<IMediaTypeDescription> sensorMediaTypeDescriptor7;
  cObjectPtr<IMediaTypeDescription> sensorMediaTypeDescriptor8;
  cObjectPtr<IMediaTypeDescription> sensorMediaTypeDescriptor9;
  cObjectPtr<IMediaTypeDescription> sensorMediaTypeOutputDescriptor;
  cObjectPtr<IMediaTypeDescription> m_pCoderDescBool;
  cObjectPtr<IMediaTypeDescription> pDescManagerObject;
  cObjectPtr<IMediaTypeDescription> object_data_description_;


  tUInt32 m_tsArduinoTime;
  tUInt32 m_f32IntervalTime;
	tFloat32 ObstacleFrontCentre_Distance;
	tFloat32 ObstacleFrontLeftCentre_Distance;
	tFloat32 ObstacleFrontRightCentre_Distance;
	tFloat32 ObstacleFrontLeft_Distance;
	tFloat32 ObstacleFrontRight_Distance;
	tFloat32 ObstacleSideLeft_Distance;
	tFloat32 ObstacleSideRight_Distance;
	tFloat32 ObstacleBackCentre_Distance;
	tFloat32 ObstacleBackRight_Distance;
	tFloat32 ObstacleBackLeft_Distance;
  bool m_bLogModeEnabled ;
  tInt counter_obstacle;

  // Thresholds
  tFloat32 SensorObjectSideThreshhold;
  tFloat32 SensorObjectFrontCenterThreshhold;

  cCriticalSection    m_oTransmitCameraObjCritSection;
  cCriticalSection    m_oTransmitSensorObjCritSection;
		
 public:
    cEmergency(const tChar* __info);
    virtual ~cEmergency();

protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);
    tResult ReadAllProperties();

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);
};

//*************************************************************************************************
#endif // _EMERGENCY_BREAK_H_
