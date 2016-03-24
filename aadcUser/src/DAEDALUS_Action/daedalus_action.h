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

1) Filter for turning performing all the action..

**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
* Updated By:: Amanullah Tariq 
**********************************************************************/

#ifndef _DAEDALUS_ACTION_FILTER_HEADER_
#define _DAEDALUS_ACTION_FILTER_HEADER_

#define OID_ADTF_DaedalusActionFilter  "adtf.daedalus.daedalus_actionfilter"

class cActionFilter : public adtf::cTimeTriggeredFilter
{
   ADTF_DECLARE_FILTER_VERSION(OID_ADTF_DaedalusActionFilter, "DAEDALUS Action Filter", OBJCAT_Tool, "DAEDALUS Action Filter", 1, 0, 1, "Version 1.0");

	protected:
    //Input Pins
	cVideoPin m_iVideoInputPin;
	cInputPin m_iDriverAction;
	cInputPin m_iWheelEncoderDistance;            // Wheel Encoder Detection Input
     
	//Output Pins
    cOutputPin m_oAccelerateOutputPin;
    cOutputPin m_oSteerOutputPin;
	cOutputPin m_oTurnSignalLeftOutputPin; 
	cOutputPin m_oTurnSignalRightOutputPin;
	cOutputPin m_oReverseIndicatorOutputPin;
	cOutputPin m_oActionPerformed;


    public: // construction
        cActionFilter(const tChar* __info);
        virtual ~cActionFilter();
		tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
		tResult RunAction();

    public: // overrides cFilter
        tResult Init(tInitStage eStage, __exception);
        
    protected: // implement cTimeTriggeredFilter
        tResult Cycle(__exception = NULL);


	private:

	int pp_counter;
	int cp_counter;
	int total_counter;

    // DDL descriptions
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescBool;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionDecisionInput;

    // arduino timestamp value
    tUInt32     m_tsArduinoTime;
        
    //Properties:
    tUInt8      m_ui8InitCtrl;
  	tFloat32 m_f32TurnLeftAngle;
	tFloat32 m_f32TurnRightAngle;
    tFloat32 m_f32StraightAngle;
    tUInt32 m_f32IntervalTime;
	
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

    tBool m_bShowLog;
    tBool m_bTurnLeft;
    tBool m_bTurnRight;
	tBool m_bReverseIndicator;
	tBool DriverModuleBufferReaderFlag;
    

    // Critical Sections
    cCriticalSection m_oTransmitSteerCritSection;
    cCriticalSection m_oTransmitAccelCritSection;
    cCriticalSection m_oTransmitRightIndicatorCritSection;
	cCriticalSection m_oTransmitLeftIndicatorCritSection;
	cCriticalSection m_oTrasmitReverseIndicatorCritSection;
    cCriticalSection m_oTransmitActionPerformedCritSection;

	//driver buffer
    tBufferID DriverNextActionBuffer; 
    tBufferID DriverTimeBuffer; 
    tBufferID DriverIsProcessedBuffer; 

	int counter_;
    bool active_;
	tInt32 DriverNextAction;
	tInt32 prevDriverAction;
	tInt32 lastCommand;

	Mat inputImage;
	bool        m_bFirstFrame;
	tBitmapFormat m_sInputFormat;

	private: // private methods
	tResult CreateOutputPins(__exception);
	tResult CreateInputPins(__exception);
    tResult ProcessInput(IMediaSample* pSample, tTimeStamp);

	/************* Transmit Methods ******************/
	tResult TransmitAcceleration(tFloat32, tTimeStamp);
    tResult TransmitSteeringAngle(const tFloat32, tTimeStamp);
    tResult TransmitRightIndicator(const tBool, tTimeStamp);
	tResult TransmitLeftIndicator(const tBool, tTimeStamp);
	tResult TransmitActionPerformed();
    tResult TransmitReverseIndicator(const tBool, tTimeStamp);

	/************* Scripts Methods ******************/
	tVoid RunParallelParkingScript();
	tVoid RunTurnLeftScript();
	tVoid RunTurnRightScript();
	tVoid RunStraightScript();
	tVoid RunCrossParkingScript();
	tVoid RunPullOverLeftScript();
	tVoid RunPullOverRightScript();
	tVoid RunPullOverLeftFromParallelParkingScript();
	tVoid RunPullOverRightFromParallelParkingScript();

	/*********** Read Properties Methods ******************/
	tResult ReadProperties();

	/************ Set Properties Methods ******************/
	tVoid SetStraightProperties();
	tVoid SetLeftTurnProperties();
	tVoid SetRightTurnProperties();
	tVoid SetCrossParkingProperties();
	tVoid SetParallelParkingProperties();
	tVoid SetPullOverLeftProperties();
	tVoid SetPullOverRightProperties();

	tVoid SetPullOverLeftFromParallelParkingProperties();
	tVoid SetPullOverRightFromParallelParkingProperties();

	/************ Get Properties Methods ******************/
	tVoid GetStraightProperties();
	tVoid GetLeftTurnProperties();
	tVoid GetRightTurnProperties();
	tVoid GetCrossParkingProperties();
	tVoid GetParallelParkingProperties();
	tVoid GetPullOverLeftProperties();
	tVoid GetPullOverRightProperties();

	tVoid GetPullOverLeftFromParallelParkingProperties();
	tVoid GetPullOverRightFromParallelParkingProperties();
	
	tVoid GenerateIPM(Mat inputImg, Mat &outputImg);
	int CrossedParking();
};


#endif // _DAEDALUS_ACTION_FILTER_HEADER_

