#include "stdafx.h"
#include "OpenChallenge.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <unistd.h>
#include <unistd.h>
#include "fcntl.h"

using namespace std;

ADTF_FILTER_PLUGIN("OpenChallenge", OID_ADTF_OpenChallenge, cOpenChallenge)
#define OC_INTERVAL_TIME "Common::Interval Time"
#define OC_AUTO_PILOT_ENABLE "Common::AutoPilot On"
#define OC_MAX_SPEED "Common::Max Speed"
#define OC_MED_SPEED "Common::Med Speed"
#define OC_MIN_SPEED "Common::Min Speed"
#define OC_MAX_LEFT_ANGLE "Common::Max Left Angle"
#define OC_MED_LEFT_ANGLE "Common::Med Left Angle"
#define OC_MIN_LEFT_ANGLE "Common::Min Left Angle"
#define OC_MAX_RIGHT_ANGLE "Common::Max Right Angle"
#define OC_MED_RIGHT_ANGLE "Common::Med Right Angle"
#define OC_MIN_RIGHT_ANGLE "Common::Min Right Angle"
#define OC_STRAIGHT_ANGLE "Common::Straight Angle"
#define OC_IP_ADDRESS "Common::IP Address"
#define OC_PORT "Common::Port"
#define OC_SHOW_LOG "Common::Show Log"


cOpenChallenge::cOpenChallenge(const tChar* __info) : adtf::cTimeTriggeredFilter(__info)
 {

  SetPropertyStr(OC_IP_ADDRESS, "192.168.1.185");
  SetPropertyInt(OC_PORT, 6700); 
  SetPropertyBool(OC_SHOW_LOG, true);
  SetPropertyFloat(OC_INTERVAL_TIME, 10000); 
  SetPropertyBool(OC_AUTO_PILOT_ENABLE, false);
  SetPropertyFloat(OC_MAX_SPEED, 1.3); 
  SetPropertyFloat(OC_MED_SPEED, 1.0); 
  SetPropertyFloat(OC_MIN_SPEED, 0.8); 
  SetPropertyFloat(OC_MAX_LEFT_ANGLE, 60.0); 
  SetPropertyFloat(OC_MED_LEFT_ANGLE, 70.0); 
  SetPropertyFloat(OC_MIN_LEFT_ANGLE, 80.0); 
  SetPropertyFloat(OC_MAX_RIGHT_ANGLE, 120.0); 
  SetPropertyFloat(OC_MED_RIGHT_ANGLE, 110.0); 
  SetPropertyFloat(OC_MIN_RIGHT_ANGLE, 100.0); 
  SetPropertyFloat(OC_STRAIGHT_ANGLE, 90.0); 
}

cOpenChallenge::~cOpenChallenge() {
}

tResult cOpenChallenge::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
  if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
  {
    InputTimeStamp = pMediaSample->GetTime();
    if(pSource==&m_iSensorFilter)
    {
      tFloat32 us_front = 0.0f;
      tFloat32 us_front_left = 0.0f;
      tFloat32 us_front_right = 0.0f;
         
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
//LOG_ERROR(cString::Format("On Pin event"));
      //TODO: Flow based threshholding for US sensor data
       LOG_ERROR(cString::Format("US Integrity: %d",(us_front != 0 || us_front_left!=0)));
      if (us_front != 0 || us_front_left!=0 || us_front_right != 0)
      {
          FlagObs= true;
      }else
      {
          FlagObs= false;
      }
    }
  }
  else if (nEventCode == IPinEventSink::PE_MediaTypeChanged && pSource != NULL)
    {
        cObjectPtr<IMediaType> pType;
        pSource->GetMediaType(&pType);
        if (pType != NULL)
        {
            cObjectPtr<IMediaTypeDescription> pMediaTypeDesc;
            RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDesc));
            if(pSource == &m_iSensorFilter)
            {
                m_pDescriptionOutputSensor = pMediaTypeDesc;
            }
        }        
    }
  RETURN_NOERROR;
}

tResult cOpenChallenge::Cycle(__exception)
{
  if(startCounter < 20)
  {
    startCounter++;
    RETURN_NOERROR;
  }
  GetMessageFromTCP();
  
  RETURN_NOERROR;
}

tResult cOpenChallenge::GetMessageFromTCP()
{
  connect(clientSocket, (struct sockaddr *) &serverAddr, addr_size);

    char buffer[5];
    /*---- Read the message from the server into the buffer ----*/
    int i = recv(clientSocket, buffer, 5, 0);
    /*---- Print the received message ----*/
    LOG_ERROR(cString::Format("Data received: %s",buffer));  

    string Message(buffer);  
  //  Message = Message.substr(0,Message.length()-1);
    ProcessInputCommand(Message);

    RETURN_NOERROR; 
}


tResult cOpenChallenge::Init(tInitStage estage, __exception) {
// -------------------------------------------------------------------------------------------------
  //RETURN_IF_FAILED(cFilter::Init(estage, __exception_ptr))
  RETURN_IF_FAILED(cTimeTriggeredFilter::Init(estage, __exception_ptr));
  
  if (estage == StageFirst) 
  { 
    CreateInputPins(__exception_ptr);
    CreateOutputPins(__exception_ptr);
  }
  else if (estage == StageNormal)
  {
    GetAllProperties();
    RETURN_IF_FAILED(SetInterval(m_f32IntervalTime));
    firstTime = true;
    FlagObs = false;
    startCounter =0;
    noCommandRecieved = 0;
    prevLeapMotionCommand = "defau";
    ObstacleSensorModuleBufferReaderFlag = tFalse;
  }
  else if (estage == StageGraphReady)
  {
  
    ConnectToTCP();
    TurnOnLights();
  }
  
  RETURN_NOERROR;
}

tVoid cOpenChallenge::ConnectToTCP()
{
  LOG_ERROR("TCP COnnected");

  /*---- Create the socket. The three arguments are: ----*/
  /* 1) Internet domain 2) Stream socket 3) Default protocol (TCP in this case) */
  clientSocket = socket(PF_INET, SOCK_STREAM, 0);
  
  /*---- Configure settings of the server address struct ----*/
  /* Address family = Internet */
  serverAddr.sin_family = AF_INET;
  /* Set port number, using htons function to use proper byte order */
  serverAddr.sin_port = htons(Port);
  /* Set IP address to localhost */
  serverAddr.sin_addr.s_addr = inet_addr(GetPropertyStr(OC_IP_ADDRESS));
  /* Set all bits of the padding field to 0 */
  memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);  

  /*---- Connect the socket to the server using the address struct ----*/
  addr_size = sizeof serverAddr;
  //connect(clientSocket, (struct sockaddr *) &serverAddr, addr_size);
}

void cOpenChallenge::GetAllProperties()
{
  showlog = GetPropertyBool(OC_SHOW_LOG);
  m_f32IntervalTime = GetPropertyInt(OC_INTERVAL_TIME);
  isAutoPilot = GetPropertyBool(OC_AUTO_PILOT_ENABLE);
  noSpeed = 0.0f;
  maxSpeed = GetPropertyFloat(OC_MAX_SPEED);
  medSpeed = GetPropertyFloat(OC_MED_SPEED);
  minSpeed = GetPropertyFloat(OC_MIN_SPEED);
  maxLeftAngle = GetPropertyFloat(OC_MAX_LEFT_ANGLE);
  medLeftAngle = GetPropertyFloat(OC_MED_LEFT_ANGLE);
  minLeftAngle = GetPropertyFloat(OC_MIN_LEFT_ANGLE);
  maxRightAngle = GetPropertyFloat(OC_MAX_RIGHT_ANGLE);
  medRightAngle = GetPropertyFloat(OC_MED_RIGHT_ANGLE);
  minRightAngle = GetPropertyFloat(OC_MIN_RIGHT_ANGLE);
  straightAngle = GetPropertyFloat(OC_STRAIGHT_ANGLE);
  Port = GetPropertyInt(OC_PORT);
}

void cOpenChallenge::StopCar()
{
  TransmitAcceleration(noSpeed,InputTimeStamp);
  TransmitSteer(straightAngle,InputTimeStamp);
  TransmitAction("stop");
}

void cOpenChallenge::TurnOffAllLights()
{
  TransmitHeadlight(tFalse,InputTimeStamp);
  TransmitLeftIndicator(tFalse,InputTimeStamp);
  TransmitRightIndicator(tFalse,InputTimeStamp);
  TransmitReverseIndicator(tFalse,InputTimeStamp);
  TransmitBrakeLights(tFalse,InputTimeStamp);
}

void cOpenChallenge::TurnOnLights()
{
  TransmitHeadlight(tTrue,InputTimeStamp);
  TransmitReverseIndicator(tTrue,InputTimeStamp);
}

bool cOpenChallenge::StopOnObstacle(string LeapMotionCommand)
{
  bool result = false;

  if(FlagObs == true && (LeapMotionCommand == "bb1s1" || LeapMotionCommand == "bb2s1"
    || LeapMotionCommand == "bb3s1" || LeapMotionCommand == "bb1l1" 
    || LeapMotionCommand == "bb2l1" || LeapMotionCommand == "bb3l1" 
    || LeapMotionCommand == "bb1l2" || LeapMotionCommand == "bb2l2" 
    || LeapMotionCommand == "bb3l2" || LeapMotionCommand == "bb1l3"  
    || LeapMotionCommand == "bb2l3" || LeapMotionCommand == "bb3l3" 
    || LeapMotionCommand == "bb1r1" || LeapMotionCommand == "bb2r1"
    || LeapMotionCommand == "bb3r1" || LeapMotionCommand == "bb1r2" 
    || LeapMotionCommand == "bb2r2" || LeapMotionCommand == "bb3r2" 
    || LeapMotionCommand == "bb1r3" || LeapMotionCommand == "bb2r3"
    || LeapMotionCommand == "bb3r3"  ))
  {
    result = false;
  }
  else if(FlagObs == true && (prevLeapMotionCommand == "bb1s1" || prevLeapMotionCommand == "bb2s1"
    || prevLeapMotionCommand == "bb3s1" || prevLeapMotionCommand == "bb1l1" 
    || prevLeapMotionCommand == "bb2l1" || prevLeapMotionCommand == "bb3l1" 
    || prevLeapMotionCommand == "bb1l2" || prevLeapMotionCommand == "bb2l2" 
    || prevLeapMotionCommand == "bb3l2" || prevLeapMotionCommand == "bb1l3"  
    || prevLeapMotionCommand == "bb2l3" || prevLeapMotionCommand == "bb3l3" 
    || prevLeapMotionCommand == "bb1r1" || prevLeapMotionCommand == "bb2r1"
    || prevLeapMotionCommand == "bb3r1" || prevLeapMotionCommand == "bb1r2" 
    || prevLeapMotionCommand == "bb2r2" || prevLeapMotionCommand == "bb3r2" 
    || prevLeapMotionCommand == "bb1r3" || prevLeapMotionCommand == "bb2r3"
    || prevLeapMotionCommand == "bb3r3"  ))
  {
    result = false;
  }
  else if (FlagObs)
  {
    result = true;
  }

  return result;
}

tVoid cOpenChallenge::ProcessInputCommand(string LeapMotionCommand)
{
    if(LeapMotionCommand == "bs1xx") //stop bs1
    {
      if (showlog) 
      {
        LOG_ERROR("stopcar");
      }
      StopCar();
      TurnOffAllLights();
    }
    else if(isAutoPilot)
    {
      if (showlog) 
      {
        LOG_ERROR("In Autopilot");
      }
      if(LeapMotionCommand ==  "bs0xx") ////stop_auto = bs0
      {
        if (showlog) 
        {
          LOG_ERROR("Stop Autopilot");
        }
        isAutoPilot = false;
        TransmitLeftIndicator(tFalse,InputTimeStamp);
        TransmitRightIndicator(tFalse,InputTimeStamp);
        StopCar();
        
        prevLeapMotionCommand = LeapMotionCommand;
      }
      else if(LeapMotionCommand == "as1xx") //auto_straight_at_cross as1
      {
        if (showlog) 
        {
          LOG_ERROR("auto_straight_at_cross");
        }
        TransmitAction("straight");
        prevLeapMotionCommand = LeapMotionCommand;
      }
      else if(LeapMotionCommand == "ar1xx") //auto_right_at_cross as1
      {
        if (showlog) 
        {
          LOG_ERROR("auto_right_at_cross");
        }
        TransmitAction("right");
        prevLeapMotionCommand = LeapMotionCommand;
      }
      else if(LeapMotionCommand == "al1xx") //auto_left_at_cross as1
      {
        if (showlog) 
        {
          LOG_ERROR("auto_left_at_cross");
        }
        TransmitAction("left");
        prevLeapMotionCommand = LeapMotionCommand;
      }

    }
    else
    {
      TurnOnLights();
      if (showlog) 
      {
        LOG_ERROR("Not In Autopilot");
      }

       if(StopOnObstacle( LeapMotionCommand))
      {
        if (showlog) 
        {
          LOG_ERROR("Obstacle Detected");
        }
          TransmitAcceleration(noSpeed,InputTimeStamp);
      }
      else 
      {

        if(LeapMotionCommand == "as0xx") //start_auto = as0
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog) 
          {
            LOG_ERROR("start_auto");
          }
          isAutoPilot = true;
          TransmitAcceleration(noSpeed,InputTimeStamp);
          TransmitSteer(straightAngle,InputTimeStamp);
          TransmitAction("straight");
          TransmitLeftIndicator(tFalse,InputTimeStamp);
          TransmitRightIndicator(tFalse,InputTimeStamp);
        }
        else if(LeapMotionCommand ==  "ba1s1") //slow_speed_straight ba1s1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog) 
          {
            LOG_ERROR("slow_speed_straight");
          }
          //TODO : Make Property
          TransmitAcceleration(minSpeed,InputTimeStamp);
          TransmitSteer(straightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba2s1") //med_speed_straight ba2s1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("med_speed_straight");
          //TODO : Make Property
          TransmitAcceleration(medSpeed,InputTimeStamp);
          TransmitSteer(straightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba3s1") //high_speed_straight ba3s1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("high_speed_straight");
          TransmitAcceleration(maxSpeed,InputTimeStamp);
          TransmitSteer(straightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb1s1") // slow_backspeed_straight bb1s1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("slow_backspeed_straight");
          TransmitAcceleration(-minSpeed,InputTimeStamp);
          TransmitSteer(straightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb2s1") // med_backspeed_straight bb2s1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("med_backspeed_straight");
          TransmitAcceleration(-medSpeed,InputTimeStamp);
          TransmitSteer(straightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb3s1") //high_backspeed_straight bb3s1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("high_backspeed_straight");
          TransmitAcceleration(-maxSpeed,InputTimeStamp);
          TransmitSteer(straightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba1l1") //slow_speed_little_left ba1l1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("slow_speed_little_left");
          TransmitAcceleration(minSpeed,InputTimeStamp);
          TransmitSteer(minLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba2l1") //med_speed_little_left ba2l1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("med_speed_little_left");
          TransmitAcceleration(medSpeed,InputTimeStamp);
          TransmitSteer(minLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba3l1") //high_speed_little_left
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("high_speed_little_left");
          TransmitAcceleration(maxSpeed,InputTimeStamp);
          TransmitSteer(minLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba1l2") //slow_speed_med_left
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("slow_speed_med_left");
          TransmitAcceleration(minSpeed,InputTimeStamp);
          TransmitSteer(medLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba2l2") //med_speed_med_left
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("med_speed_med_left");
          TransmitAcceleration(medSpeed,InputTimeStamp);
          TransmitSteer(medLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba3l2") //high_speed_med_left ba3l2
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("high_speed_med_left");
          TransmitAcceleration(medSpeed,InputTimeStamp);
          TransmitSteer(medLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba1l3") //mslow_speed_hard_left ba1l3
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("mslow_speed_hard_left");
          TransmitAcceleration(minSpeed,InputTimeStamp);
          TransmitSteer(maxLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba2l3") //med_speed_hard_left ba2l3
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("med_speed_hard_left");
          TransmitAcceleration(medSpeed,InputTimeStamp);
          TransmitSteer(maxLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba3l3") //high_speed_hard_left ba3l3
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("high_speed_hard_left");
          TransmitAcceleration(maxSpeed,InputTimeStamp);
          TransmitSteer(maxLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba1r1") //slow_speed_little_right ba1r1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("slow_speed_little_right");
          TransmitAcceleration(minSpeed,InputTimeStamp);
          TransmitSteer(minRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba2r1") //med_speed_little_right ba2r1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("med_speed_little_right");
          TransmitAcceleration(medSpeed,InputTimeStamp);
          TransmitSteer(minRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba3r1") //high_speed_little_right ba3r1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("high_speed_little_right");
          TransmitAcceleration(maxSpeed,InputTimeStamp);
          TransmitSteer(minRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba1r2") //slow_speed_med_right ba1r2
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("slow_speed_med_right");
          TransmitAcceleration(minSpeed,InputTimeStamp);
          TransmitSteer(medRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba2r2") //med_speed_med_right ba2r2
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("med_speed_med_right");
          TransmitAcceleration(medSpeed,InputTimeStamp);
          TransmitSteer(medRightAngle,InputTimeStamp);
        
        }
        else if(LeapMotionCommand == "ba3r2") //high_speed_med_right ba3r2
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("high_speed_med_right");
          TransmitAcceleration(maxSpeed,InputTimeStamp);
          TransmitSteer(medRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba1r3") //slow_speed_hard_right ba1r3
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("slow_speed_hard_right");
          TransmitAcceleration(minSpeed,InputTimeStamp);
          TransmitSteer(maxRightAngle,InputTimeStamp);
         // TransmitRightIndicator(tTrue,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba2r3") //med_speed_hard_right ba2r3
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("med_speed_hard_right");
          TransmitAcceleration(medSpeed,InputTimeStamp);
          TransmitSteer(maxRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "ba3r3") //high_speed_hard_right ba3r3
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("high_speed_hard_right");
          TransmitAcceleration(maxSpeed,InputTimeStamp);
          TransmitSteer(maxRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb1l1") //slow_backspeed_little_left bb1l1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("slow_backspeed_little_left");
          TransmitAcceleration(-minSpeed,InputTimeStamp);
          TransmitSteer(minRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb2l1") //med_backspeed_little_left bb2l1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("med_backspeed_little_left");
          TransmitAcceleration(-medSpeed,InputTimeStamp);
          TransmitSteer(minRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb3l1") //high_backspeed_little_left bb3l1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("high_backspeed_little_left");
          TransmitAcceleration(-maxSpeed,InputTimeStamp);
          TransmitSteer(minRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb1l2") //slow_backspeed_med_left bb1l2
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("slow_backspeed_med_left");
          TransmitAcceleration(-minSpeed,InputTimeStamp);
          TransmitSteer(medRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb2l2") //med_backspeed_med_left bb2l2
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("med_backspeed_med_left");
          TransmitAcceleration(-medSpeed,InputTimeStamp);
          TransmitSteer(medRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb3l2") //high_backspeed_med_left bb3l2
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("high_backspeed_med_left");
          TransmitAcceleration(-maxSpeed,InputTimeStamp);
          TransmitSteer(medRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb1l3") //slow_backspeed_hard_left bb1l3
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("slow_backspeed_hard_left");
          TransmitAcceleration(-minSpeed,InputTimeStamp);
          TransmitSteer(maxRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb2l3") //med_backspeed_hard_left bb2l3
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("med_backspeed_hard_left");
          TransmitAcceleration(-medSpeed,InputTimeStamp);
          TransmitSteer(maxRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb3l3") //high_backspeed_hard_left bb3l3
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("high_backspeed_hard_left");
          TransmitAcceleration(-maxSpeed,InputTimeStamp);
          TransmitSteer(maxRightAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb1r1") //slow_backspeed_little_right bb1r1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("slow_backspeed_little_right");
          TransmitAcceleration(-minSpeed,InputTimeStamp);
          TransmitSteer(minLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb2r1") //med_backspeed_little_right bb2r1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("med_backspeed_little_right");
          TransmitAcceleration(-medSpeed,InputTimeStamp);
          TransmitSteer(minLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb3r1") //high_backspeed_little_right bb3r1
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("high_backspeed_little_right");
          TransmitAcceleration(-maxSpeed,InputTimeStamp);
          TransmitSteer(minLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb1r2") //slow_backspeed_med_right bb1r2
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("slow_backspeed_med_right");
          TransmitAcceleration(-minSpeed,InputTimeStamp);
          TransmitSteer(medLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb2r2") //med_backspeed_med_right bb2r2
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("med_backspeed_med_right");
          TransmitAcceleration(-medSpeed,InputTimeStamp);
          TransmitSteer(medLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb3r2") //high_backspeed_med_right bb3r2
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("high_backspeed_med_right");
          TransmitAcceleration(-maxSpeed,InputTimeStamp);
          TransmitSteer(medLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb1r3") //slow_backspeed_hard_right bb1r3
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("slow_backspeed_hard_right");
          TransmitAcceleration(-minSpeed,InputTimeStamp);
          TransmitSteer(maxLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb2r3") //med_backspeed_hard_right bb2r3
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
            LOG_ERROR("med_backspeed_hard_right");
          TransmitAcceleration(-medSpeed,InputTimeStamp);
          TransmitSteer(maxLeftAngle,InputTimeStamp);
        }
        else if(LeapMotionCommand == "bb3r3") //high_backspeed_hard_right bb3r3
        {
          prevLeapMotionCommand = LeapMotionCommand;
          if (showlog)
           LOG_ERROR("high_backspeed_hard_right");
          TransmitAcceleration(-maxSpeed,InputTimeStamp);
          TransmitSteer(maxLeftAngle,InputTimeStamp);
        }
        else //Nothing Received
        {
          noCommandRecieved ++;
        }
      }
    }
  
}

tResult cOpenChallenge::CreateInputPins(__exception)
{
  cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
  tChar const * output_stream_type2 = pDescManager->GetMediaDescription("tObstacleDistanceStruct");
  RETURN_IF_POINTER_NULL(output_stream_type2);
  cObjectPtr<IMediaType> output_type_signal_value2 = new cMediaType(0, 0, 0, "tObstacleDistanceStruct", output_stream_type2,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
  RETURN_IF_FAILED(output_type_signal_value2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSensor));
  
  RETURN_IF_FAILED(m_iSensorFilter.Create("Emergency_Sensor", output_type_signal_value2, static_cast<IPinEventSink*> (this)));    
  RETURN_IF_FAILED(RegisterPin(&m_iSensorFilter));

  RETURN_NOERROR;
}

tResult cOpenChallenge::CreateOutputPins(__exception)
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
    

    // Indicator Output pin
    RETURN_IF_FAILED(m_oTurnSignalLeftOutputPin.Create("turnSignalLeftEnabled", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oTurnSignalLeftOutputPin));

    RETURN_IF_FAILED(m_oTurnSignalRightOutputPin.Create("turnSignalRightEnabled", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oTurnSignalRightOutputPin));

 
    // Brakelight Output pin
    RETURN_IF_FAILED(m_oBrakeLightOutputPin.Create("brakeLightEnabled", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oBrakeLightOutputPin));

    // Reverse Output pin
    RETURN_IF_FAILED(m_oReverseIndicatorOutputPin.Create("ReverseLightEnabled", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oReverseIndicatorOutputPin));

  // inputs for the decision module
     tChar const * strDescIdValue = pDescManager->GetMediaDescription("tIdValue");    
     RETURN_IF_POINTER_NULL(strDescIdValue);      
   
   cObjectPtr<IMediaType> pTypeIdValue = new cMediaType(0, 0, 0, "tIdValue", strDescIdValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
     RETURN_IF_FAILED(pTypeIdValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionDecisionOutput));    
        
     RETURN_IF_FAILED(m_DecisionOutputPin.Create("Action",pTypeIdValue, static_cast<IPinEventSink*> (this)));
     RETURN_IF_FAILED(RegisterPin(&m_DecisionOutputPin));
    
    RETURN_NOERROR;
}

tResult cOpenChallenge::TransmitSteer(const tFloat32 Steer, tTimeStamp tsInputTime)
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
 
          pCoderOutput->Set("f32Value", (tVoid*)&Steer);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
              
    pNewMediaSample->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oSteerOutputPin.Transmit(pNewMediaSample));
    RETURN_NOERROR;
}

tResult cOpenChallenge::TransmitAcceleration(tFloat32 Acceleration, tTimeStamp tsInputTime)
{
    
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

tResult cOpenChallenge::TransmitRightIndicator(const tBool bTurnRightIndicator, tTimeStamp tsInputTime)
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

tResult cOpenChallenge::TransmitLeftIndicator(const tBool bTurnLeftIndicator, tTimeStamp tsInputTime)
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


tResult cOpenChallenge::TransmitReverseIndicator(const tBool bTurnOnIndicator, tTimeStamp tsInputTime)
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

tResult cOpenChallenge::TransmitHeadlight(const tBool bHeadLights, tTimeStamp tsInputTime)
{
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

tResult cOpenChallenge::TransmitBrakeLights(const tBool bHeadLights, tTimeStamp tsInputTime)
{
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

tResult cOpenChallenge::TransmitAction(cString m_sCurrentManeuver)
{
  __synchronized_obj(m_oTransmitActionCritSection);
   cObjectPtr<IMediaSample> pMediaSample;
   RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

   cObjectPtr<IMediaSerializer> pSerializer;
   m_pDescriptionDecisionOutput->GetMediaSampleSerializer(&pSerializer);
   tInt nSize = pSerializer->GetDeserializedSize();
   
   DAEDALUS::Maneuver eManeuver;
   if (m_sCurrentManeuver == "left")
         eManeuver = DAEDALUS::left;
   else if (m_sCurrentManeuver == "right")
      eManeuver = DAEDALUS::right;
   else if (m_sCurrentManeuver == "straight")
      eManeuver = DAEDALUS::straight;
   else if (m_sCurrentManeuver == "cross_parking")
      eManeuver = DAEDALUS::cross_parking;
   else if (m_sCurrentManeuver == "parallel_parking")
      eManeuver = DAEDALUS::parallel_parking;
   else if (m_sCurrentManeuver == "pull_out_left")
      eManeuver = DAEDALUS::pull_out_left;
   else if (m_sCurrentManeuver == "pull_out_right")
      eManeuver = DAEDALUS::pull_out_right;
   else if(m_sCurrentManeuver == "stop")
     eManeuver = DAEDALUS::stop;
   
   tBool bIsExecuted = false;
   tFloat32 timestamp = 0;

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {   // scope for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionDecisionOutput, pMediaSample, pCoder);

      pCoder->Set("ui32Timestamp", (tVoid*)&timestamp);
        pCoder->Set("i32Value", (tVoid*)&eManeuver);
        pCoder->Set("bIsExecuted", (tVoid*)&bIsExecuted);
    }      
        
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_DecisionOutputPin.Transmit(pMediaSample);
   
   RETURN_NOERROR;
}