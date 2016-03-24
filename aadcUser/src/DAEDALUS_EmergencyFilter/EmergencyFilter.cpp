#include "stdafx.h"
#include "EmergencyFilter.h"
// #include <fstream>
// #include <sstream>
// #include <iostream>
#include "daedalus_obstacle_enum.h"

using namespace cv;
// using namespace cvb;



ADTF_FILTER_PLUGIN("DAEDALUS_emergency", OID_ADTF_DAEDALUS_EMERGENCY, cEmergency)
#define AF_INTERVAL_TIME "Common::Interval Time"
// -------------------------------------------------------------------------------------------------
cEmergency::cEmergency(const tChar* __info) : adtf::cTimeTriggeredFilter(__info) {
  
  SetPropertyFloat("SensorObjectFrontCenterThreshhold",0.5);
  SetPropertyFloat("SensorObjectFrontCenterRightThreshhold",0.5);
  SetPropertyFloat("SensorObjectFrontCenterLeftThreshhold",0.5);
  SetPropertyFloat("SensorObjectFrontLeftThreshhold",0.5);
  SetPropertyFloat("SensorObjectFrontRightThreshhold",0.5);
  SetPropertyFloat("SensorObjectSideThreshhold",0.5);
  SetPropertyFloat("SensorObjectBackCenterThreshhold",0.5);
  SetPropertyFloat("SensorObjectBackRightThreshhold",0.5);
  SetPropertyFloat("SensorObjectBackLeftThreshhold",0.5);
  SetPropertyBool("m_bLogModeEnabled", false);  
  SetPropertyInt("SensorObjectCounterThreshhold",100);
  SetPropertyFloat(AF_INTERVAL_TIME, 20000); 

}

cEmergency::~cEmergency() {
}

tResult cEmergency::CreateInputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
    tChar const* strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pType = new cMediaType(0,0,0,"tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    
    RETURN_IF_FAILED(m_usSensorFront.Create("UltraSound_Front", pType, this));
    RETURN_IF_FAILED(RegisterPin(&m_usSensorFront));
    RETURN_IF_FAILED(m_usSensorFrontLeft.Create("UltraSound_Front_Left", pType, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_usSensorFrontLeft));
    RETURN_IF_FAILED(m_usSensorFrontRight.Create("UltraSound_Front_Right", pType, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_usSensorFrontRight));
    RETURN_IF_FAILED(m_usSensorFrontLeftCenter.Create("UltraSound_Front_Left_Center", pType, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_usSensorFrontLeftCenter));
    RETURN_IF_FAILED(m_usSensorFrontRightCenter.Create("UltraSound_Front_Right_Center", pType, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_usSensorFrontRightCenter));
    
    RETURN_IF_FAILED(m_usSensorSideLeft.Create("UltraSound_Side_Left", pType, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_usSensorSideLeft));
    RETURN_IF_FAILED(m_usSensorSideRight.Create("UltraSound_Side_Right", pType, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_usSensorSideRight));
    RETURN_IF_FAILED(m_usSensorRearLeft.Create("UltraSound_Back_Left", pType, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_usSensorRearLeft));
    RETURN_IF_FAILED(m_usSensorRearCenter.Create("UltraSound_Back_Center", pType, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_usSensorRearCenter));
    RETURN_IF_FAILED(m_usSensorRearRight.Create("UltraSound_Back_Right", pType, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_usSensorRearRight));

    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&sensorMediaTypeDescriptor0));
    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&sensorMediaTypeDescriptor1));
    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&sensorMediaTypeDescriptor2));
    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&sensorMediaTypeDescriptor3));
    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&sensorMediaTypeDescriptor4));
    
    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&sensorMediaTypeDescriptor5));
    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&sensorMediaTypeDescriptor6));
    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&sensorMediaTypeDescriptor7));
    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&sensorMediaTypeDescriptor8));
    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&sensorMediaTypeDescriptor9));

 RETURN_NOERROR;
}

tResult cEmergency::CreateOutputPins(__exception)
{

    cObjectPtr<IMediaDescriptionManager> pSensorOutputManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pSensorOutputManager, __exception_ptr));
    tChar const* strpSensorOutputManager = pSensorOutputManager->GetMediaDescription("tObstacleDistanceStruct");
    RETURN_IF_POINTER_NULL(strpSensorOutputManager);
    cObjectPtr<IMediaType> SensorOutputType = new cMediaType(0,0,0,"tObstacleDistanceStruct", strpSensorOutputManager, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
  
    RETURN_IF_FAILED(m_sensor_obstacle_value.Create("US_Sensor_Output", SensorOutputType, this));
    RETURN_IF_FAILED(RegisterPin(&m_sensor_obstacle_value));
    RETURN_IF_FAILED(SensorOutputType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&sensorMediaTypeOutputDescriptor));
 

 
 RETURN_NOERROR;
}

tResult cEmergency::Cycle(__exception)
{
	  tFloat32 max = 99.99f;
      if(ObstacleSideLeft_Distance > GetPropertyFloat("SensorObjectSideThreshhold") || ObstacleSideLeft_Distance == 0.0)  ObstacleSideLeft_Distance = max;
      if(ObstacleFrontCentre_Distance > GetPropertyFloat("SensorObjectFrontCenterThreshhold") || ObstacleFrontCentre_Distance == 0.0)  ObstacleFrontCentre_Distance = max;
      if(ObstacleFrontRightCentre_Distance > GetPropertyFloat("SensorObjectFrontCenterRightThreshhold") || ObstacleFrontRightCentre_Distance == 0.0)  ObstacleFrontRightCentre_Distance = max;
      if(ObstacleFrontLeftCentre_Distance > GetPropertyFloat("SensorObjectFrontCenterLeftThreshhold") || ObstacleFrontLeftCentre_Distance == 0.0)  ObstacleFrontLeftCentre_Distance = max;
      if(ObstacleFrontLeft_Distance > GetPropertyFloat("SensorObjectFrontLeftThreshhold") || ObstacleFrontLeft_Distance == 0.0)  ObstacleFrontLeft_Distance = max;
      if(ObstacleFrontRight_Distance > GetPropertyFloat("SensorObjectFrontRightThreshhold") || ObstacleFrontRight_Distance == 0.0)  ObstacleFrontRight_Distance = max;
      if(ObstacleSideLeft_Distance > GetPropertyFloat("SensorObjectSideThreshhold") || ObstacleSideLeft_Distance == 0.0)  ObstacleSideLeft_Distance = max;
      if(ObstacleSideRight_Distance > GetPropertyFloat("SensorObjectSideThreshhold") || ObstacleSideRight_Distance == 0.0)  ObstacleSideRight_Distance = max;
      if(ObstacleBackLeft_Distance > GetPropertyFloat("SensorObjectBackLeftThreshhold") || ObstacleBackLeft_Distance == 0.0)  ObstacleBackLeft_Distance = max;
      if(ObstacleBackCentre_Distance > GetPropertyFloat("SensorObjectBackCenterThreshhold") || ObstacleBackCentre_Distance == 0.0)  ObstacleBackCentre_Distance = max;
      if(ObstacleBackRight_Distance > GetPropertyFloat("SensorObjectBackRightThreshhold") || ObstacleBackRight_Distance == 0.0)  ObstacleBackRight_Distance = max;
      ProcessObjectSensor();
      
      RETURN_NOERROR;
}

// -------------------------------------------------------------------------------------------------
tResult cEmergency::Init(tInitStage estage, __exception) {
// -------------------------------------------------------------------------------------------------
  //RETURN_IF_FAILED(cFilter::Init(estage, __exception_ptr))
  RETURN_IF_FAILED(cTimeTriggeredFilter::Init(estage, __exception_ptr));
  
  if (estage == StageFirst) {  
    CreateInputPins(__exception_ptr);
    CreateOutputPins(__exception_ptr);
  }
  else if (estage == StageNormal)
	{
    counter_obstacle=0;
    m_f32IntervalTime = GetPropertyInt(AF_INTERVAL_TIME);
	 ObstacleFrontCentre_Distance =0.0;
	 ObstacleFrontLeftCentre_Distance=0.0;
	 ObstacleFrontRightCentre_Distance=0.0;
	 ObstacleFrontLeft_Distance=0.0;
	 ObstacleFrontRight_Distance=0.0;
	 ObstacleSideLeft_Distance=0.0;
	 ObstacleSideRight_Distance=0.0;
	 ObstacleBackCentre_Distance=0.0;
	 ObstacleBackRight_Distance=0.0;
	 ObstacleBackLeft_Distance=0.0;
   RETURN_IF_FAILED(SetInterval(m_f32IntervalTime));
	}
  else if (estage == StageGraphReady) {
    
  }
  
  RETURN_NOERROR;
}

// -------------------------------------------------------------------------------------------------
tResult cEmergency::Shutdown(tInitStage eStage, __exception) {
  return cFilter::Shutdown(eStage,__exception_ptr);
}

// -------------------------------------------------------------------------------------------------
tResult cEmergency::OnPinEvent(IPin* source, tInt event_code, tInt param1, tInt param2,
    IMediaSample* pMediaSample) {
	//LOG_ERROR(cString::Format("EmergencyFilter: On in pin event "));
  RETURN_IF_POINTER_NULL(source);
  RETURN_IF_POINTER_NULL(pMediaSample);
// -------------------------------------------------------------------------------------------------
  if (event_code == IPinEventSink::PE_MediaSampleReceived && source != NULL)

    {
     // tFloat32 max = 99.99f;
      m_bLogModeEnabled=GetPropertyBool("m_bLogModeEnabled");
      // if (m_bLogModeEnabled)     LOG_ERROR(cString::Format("EmergencyFilter: event received "));

      tFloat32 _sensor_value = 0.0f;
      if (source==&m_usSensorFront ){
          {   
             //  if (m_bLogModeEnabled)     LOG_ERROR(cString::Format("EmergencyFilter: Front Sensor "));
              __adtf_sample_read_lock_mediadescription(sensorMediaTypeDescriptor0,pMediaSample,pCoderInput);       
              pCoderInput->Get("ui32ArduinoTimestamp", &m_tsArduinoTime);
              pCoderInput->Get("f32Value", &_sensor_value);  
          }
          ObstacleFrontCentre_Distance=_sensor_value;
          //if(ObstacleFrontCentre_Distance > GetPropertyFloat("SensorObjectFrontCenterThreshhold") || ObstacleFrontCentre_Distance == 0.0)  ObstacleFrontCentre_Distance = max;
          //ProcessObjectSensor();
      }
  		else if (source==&m_usSensorFrontRightCenter){
  		    {   
             // if (m_bLogModeEnabled)     LOG_ERROR(cString::Format("EmergencyFilter: Front right centre Sensor "));
              __adtf_sample_read_lock_mediadescription(sensorMediaTypeDescriptor1,pMediaSample,pCoderInput);       
              pCoderInput->Get("ui32ArduinoTimestamp", &m_tsArduinoTime);
              pCoderInput->Get("f32Value", &_sensor_value);  
          }
          
          ObstacleFrontRightCentre_Distance=_sensor_value;
          //if(ObstacleFrontRightCentre_Distance > GetPropertyFloat("SensorObjectFrontCenterRightThreshhold") || ObstacleFrontRightCentre_Distance == 0.0)  ObstacleFrontRightCentre_Distance = max;
          //ProcessObjectSensor();
  		}
  		else if (source==&m_usSensorFrontLeftCenter){
  			  {   
             // if (m_bLogModeEnabled)     LOG_ERROR(cString::Format("EmergencyFilter: Front Left centre Sensor "));
              __adtf_sample_read_lock_mediadescription(sensorMediaTypeDescriptor2,pMediaSample,pCoderInput);       
              pCoderInput->Get("ui32ArduinoTimestamp", &m_tsArduinoTime);
              pCoderInput->Get("f32Value", &_sensor_value);  
          }
          ObstacleFrontLeftCentre_Distance=_sensor_value;
          //if(ObstacleFrontLeftCentre_Distance > GetPropertyFloat("SensorObjectFrontCenterLeftThreshhold") || ObstacleFrontLeftCentre_Distance == 0.0)  ObstacleFrontLeftCentre_Distance = max;
          
          //ProcessObjectSensor();
  		}
      else if (source==&m_usSensorFrontLeft){
          {   
              //if (m_bLogModeEnabled)     LOG_ERROR(cString::Format("EmergencyFilter: Front Left Sensor "));
              __adtf_sample_read_lock_mediadescription(sensorMediaTypeDescriptor3,pMediaSample,pCoderInput);       
              pCoderInput->Get("ui32ArduinoTimestamp", &m_tsArduinoTime);
              pCoderInput->Get("f32Value", &_sensor_value);  
          }
          ObstacleFrontLeft_Distance=_sensor_value;
          //if(ObstacleFrontLeft_Distance > GetPropertyFloat("SensorObjectFrontLeftThreshhold") || ObstacleFrontLeft_Distance == 0.0)  ObstacleFrontLeft_Distance = max;
          
          //ProcessObjectSensor();
      }
      else if (source==&m_usSensorFrontRight){
          {   
              //if (m_bLogModeEnabled)     LOG_ERROR(cString::Format("EmergencyFilter: Front Right Sensor "));
              __adtf_sample_read_lock_mediadescription(sensorMediaTypeDescriptor4,pMediaSample,pCoderInput);       
              pCoderInput->Get("ui32ArduinoTimestamp", &m_tsArduinoTime);
              pCoderInput->Get("f32Value", &_sensor_value);  
          }

          
          ObstacleFrontRight_Distance=_sensor_value;
          //if(ObstacleFrontRight_Distance > GetPropertyFloat("SensorObjectFrontRightThreshhold") || ObstacleFrontRight_Distance == 0.0)  ObstacleFrontRight_Distance = max;
           
          //ProcessObjectSensor();
      }
      
      else if (source==&m_usSensorSideLeft){
          {   
              //if (m_bLogModeEnabled)     LOG_ERROR(cString::Format("EmergencyFilter: side left Sensor "));
              __adtf_sample_read_lock_mediadescription(sensorMediaTypeDescriptor5,pMediaSample,pCoderInput);       
              pCoderInput->Get("ui32ArduinoTimestamp", &m_tsArduinoTime);
              pCoderInput->Get("f32Value", &_sensor_value);  
          }
          ObstacleSideLeft_Distance=_sensor_value;
          //if(ObstacleSideLeft_Distance > GetPropertyFloat("SensorObjectSideThreshhold") || ObstacleSideLeft_Distance == 0.0)  ObstacleSideLeft_Distance = max;
          //ProcessObjectSensor();
      }
  		else if (source==&m_usSensorSideRight){
   		    {   
              //if (m_bLogModeEnabled)     LOG_ERROR(cString::Format("EmergencyFilter: side right Sensor "));
              __adtf_sample_read_lock_mediadescription(sensorMediaTypeDescriptor6,pMediaSample,pCoderInput);       
              pCoderInput->Get("ui32ArduinoTimestamp", &m_tsArduinoTime);
              pCoderInput->Get("f32Value", &_sensor_value);  
          }
          ObstacleSideRight_Distance=_sensor_value;
          //if(ObstacleSideRight_Distance > GetPropertyFloat("SensorObjectSideThreshhold") || ObstacleSideRight_Distance == 0.0)  ObstacleSideRight_Distance = max;
          //ProcessObjectSensor();
  		}
      else if (source==&m_usSensorRearLeft){
          {   
              //if (m_bLogModeEnabled)     LOG_ERROR(cString::Format("EmergencyFilter: back left sensor "));
              __adtf_sample_read_lock_mediadescription(sensorMediaTypeDescriptor7,pMediaSample,pCoderInput);       
              pCoderInput->Get("ui32ArduinoTimestamp", &m_tsArduinoTime);
              pCoderInput->Get("f32Value", &_sensor_value);  
          }
          ObstacleBackLeft_Distance=_sensor_value;
          //if(ObstacleBackLeft_Distance > GetPropertyFloat("SensorObjectBackLeftThreshhold") || ObstacleBackLeft_Distance == 0.0)  ObstacleBackLeft_Distance = max;
          //ProcessObjectSensor();
      }
    	else if (source==&m_usSensorRearCenter){
    	    {   
             // if (m_bLogModeEnabled)     LOG_ERROR(cString::Format("EmergencyFilter: back centre Sensor "));
              __adtf_sample_read_lock_mediadescription(sensorMediaTypeDescriptor8,pMediaSample,pCoderInput);       
              pCoderInput->Get("ui32ArduinoTimestamp", &m_tsArduinoTime);
              pCoderInput->Get("f32Value", &_sensor_value);  
          }
          ObstacleBackCentre_Distance=_sensor_value;
          //if(ObstacleBackCentre_Distance > GetPropertyFloat("SensorObjectBackCenterThreshhold") || ObstacleBackCentre_Distance == 0.0)  ObstacleBackCentre_Distance = max;
            //  ProcessObjectSensor();
    	}
    	else if(source==&m_usSensorRearRight){
    	    {   
             // if (m_bLogModeEnabled)     LOG_ERROR(cString::Format("EmergencyFilter: back right Sensor "));
              __adtf_sample_read_lock_mediadescription(sensorMediaTypeDescriptor9,pMediaSample,pCoderInput);       
              pCoderInput->Get("ui32ArduinoTimestamp", &m_tsArduinoTime);
              pCoderInput->Get("f32Value", &_sensor_value);  
          }
          ObstacleBackRight_Distance=_sensor_value;
          //if(ObstacleBackRight_Distance > GetPropertyFloat("SensorObjectBackRightThreshhold") || ObstacleBackRight_Distance == 0.0)  ObstacleBackRight_Distance = max;
          //ProcessObjectSensor();
    	}

    }else if (event_code == IPinEventSink::PE_MediaTypeChanged && source != NULL)
    {
        cObjectPtr<IMediaType> pType;
        source->GetMediaType(&pType);
        if (pType != NULL)
        {
            cObjectPtr<IMediaTypeDescription> pMediaTypeDesc;
            RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDesc));
            if (source==&m_usSensorFront ){
                sensorMediaTypeDescriptor0 = pMediaTypeDesc;
            }
            else if (source==&m_usSensorFrontRightCenter){
                sensorMediaTypeDescriptor1 = pMediaTypeDesc;
            }
            else if (source==&m_usSensorFrontLeftCenter){
                sensorMediaTypeDescriptor2 = pMediaTypeDesc;
            }
            else if (source==&m_usSensorFrontLeft){
                sensorMediaTypeDescriptor3 = pMediaTypeDesc;
            }
            else if (source==&m_usSensorFrontRight){
                sensorMediaTypeDescriptor4 = pMediaTypeDesc;
            }
            else if (source==&m_usSensorSideLeft){
                sensorMediaTypeDescriptor5 = pMediaTypeDesc;
            }
            else if (source==&m_usSensorSideRight){
                sensorMediaTypeDescriptor6 = pMediaTypeDesc;
            }
            else if (source==&m_usSensorRearLeft){
                sensorMediaTypeDescriptor7 = pMediaTypeDesc;
            }
            else if (source==&m_usSensorRearCenter){
                sensorMediaTypeDescriptor8 = pMediaTypeDesc;
            }
            else if(source==&m_usSensorRearRight){
                sensorMediaTypeDescriptor9 = pMediaTypeDesc;
            }
        }        
    }
      RETURN_NOERROR;
}


        

tResult cEmergency::ProcessObjectSensor()
{
   
    tFloat32 max = 99.9f;
    
    
    if(ObstacleFrontRight_Distance == max && 
        ObstacleFrontLeft_Distance == max && 
        ObstacleFrontCentre_Distance == max && 
        ObstacleFrontLeftCentre_Distance == max && 
        ObstacleFrontRightCentre_Distance == max &&
        ObstacleSideLeft_Distance == max &&
        ObstacleSideRight_Distance == max &&
        ObstacleBackCentre_Distance == max &&
        ObstacleBackLeft_Distance == max &&
        ObstacleBackRight_Distance == max)
          RETURN_NOERROR;

    tFloat32 _FrontCentre = 0.0f;  
    tFloat32 _FrontLeft = 0.0f; 
    tFloat32 _FrontRight = 0.0f; 
    tFloat32 _Left = 0.0f; 
    tFloat32 _Right = 0.0f; 
    tFloat32 _Back = 0.0f; 

    _FrontCentre = std::min(std::min(ObstacleFrontLeftCentre_Distance,ObstacleFrontRightCentre_Distance),ObstacleFrontCentre_Distance);
    _FrontLeft = std::min(ObstacleFrontLeftCentre_Distance,ObstacleFrontLeft_Distance);
    _FrontRight = std::min(ObstacleFrontRight_Distance,ObstacleFrontRightCentre_Distance);
    _Left = ObstacleSideLeft_Distance;
    _Right = ObstacleSideRight_Distance; 
    _Back = std::min(std::min(ObstacleBackLeft_Distance,ObstacleBackRight_Distance),ObstacleBackCentre_Distance);
    if(_FrontCentre >= max)  _FrontCentre = 0.0f;
    if(_FrontLeft >= max)  _FrontLeft = 0.0f;
    if(_FrontRight >= max)  _FrontRight = 0.0f;
    if(_Left >= max)  _Left = 0.0f;
    if(_Right >= max)  _Right = 0.0f;
    if(_Back >= max)  _Back = 0.0f;


    TransmitSensorObjectInfoToDecisionModule(_FrontCentre,_FrontLeft,_FrontRight,_Left,_Right,_Back);
	RETURN_NOERROR;
}


tResult cEmergency::TransmitSensorObjectInfoToDecisionModule(tFloat32 F,tFloat32 FL, tFloat32 FR, tFloat32 L, tFloat32 R, tFloat32 B)
{   
    if (m_bLogModeEnabled)
    {
	   LOG_ERROR(cString::Format("EmergencyFilter: obstacle Distance -> Front : %f",  F));
     LOG_ERROR(cString::Format("EmergencyFilter: obstacle Distance -> FRONT LEFT : %f",  FL));
     LOG_ERROR(cString::Format("EmergencyFilter: obstacle Distance -> Front RIGHT: %f",  FR));
     LOG_ERROR(cString::Format("EmergencyFilter: obstacle Distance -> LEFT : %f",  L));
     LOG_ERROR(cString::Format("EmergencyFilter: obstacle Distance -> RIGHT : %f",  R));
     LOG_ERROR(cString::Format("EmergencyFilter: obstacle Distance -> Back : %f",  B));
    }
    __synchronized_obj(m_oTransmitSensorObjCritSection);
    cObjectPtr<IMediaSample> _mediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&_mediaSample));
   
    cObjectPtr<IMediaCoder> pCoder;
    cObjectPtr<IMediaSerializer> pSerializer;
    sensorMediaTypeOutputDescriptor->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    
    RETURN_IF_FAILED(_mediaSample->AllocBuffer(nSize));
    //if (m_bLogModeEnabled)     LOG_ERROR(cString::Format("EmergencyFilter: Data sending started "));
    
    {
    __adtf_sample_write_lock_mediadescription(sensorMediaTypeOutputDescriptor, _mediaSample, pCoder);    
    pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);
    pCoder->Set("minimum_dist[0]", (tVoid*)&F);
    pCoder->Set("minimum_dist[1]", (tVoid*)&FL);
    pCoder->Set("minimum_dist[2]", (tVoid*)&FR);
    pCoder->Set("minimum_dist[3]", (tVoid*)&L);
    pCoder->Set("minimum_dist[4]", (tVoid*)&R);
    pCoder->Set("minimum_dist[5]", (tVoid*)&B);
    }

   _mediaSample->SetTime(_clock->GetStreamTime());
    
    m_sensor_obstacle_value.Transmit(_mediaSample);
	RETURN_NOERROR;
}

