/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************/
#ifndef _LaneTrackingExt_FILTER_HEADER_
#define _LaneTrackingExt_FILTER_HEADER_

#define OID_ADTF_LaneTrackingExt  "adtf.aadc.aadc_laneTracking_ext"

class cLaneTrackingExt : public adtf::cFilter,
                      public adtf::ISignalProvider
{
    
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_LaneTrackingExt, "DAEDALUS LaneTrackingExt", OBJCAT_Tool, "AADC Lane Tracking Extended", 1, 0, 1, "BFFT GmbH");    
    
protected:
    //Eingang f�r RGB Bild
    cVideoPin           m_oVideoInputPin;
    cInputPin           m_oInputStart; 
        
    cOutputPin          m_oGCLOutput;
	cOutputPin			m_oLaneTrackingOutput;

public:

    struct sPoint 
    {
        tInt16 x;
        tInt16 y;
        tBool bIsValid;
        sPoint()
        {
            x = 0;
            y = 0;
            bIsValid = tTrue;
        }
        sPoint(tBool bIsValid)
        {
            x = 0;
            y = 0;
            this->bIsValid = bIsValid;
        }
        sPoint(tInt16 x, tInt16 y, tBool bIsValid)
        {
            this->x = x;
            this->y = y;
            this->bIsValid = bIsValid;
        }
    };

                
    cLaneTrackingExt(const tChar*);
    virtual ~cLaneTrackingExt();

    // implements cFilter
    tResult Init(tInitStage eStage, __exception=NULL);
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr=NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult PropertyChanged(const char* strProperty);

public: // implements ISignalProvider
    virtual tResult GetSignalValue(tSignalID nSignalID, tSignalValue* pValue);
    virtual tResult ActivateSignalEvents(tSignalID nSignalID, tTimeStamp nUpdateRate = 0);
    virtual tResult DeactivateSignalEvents(tSignalID nSignalID);

public: // implements IObject
    tResult GetInterface(const tChar* idInterface, tVoid** ppvObject);
    tUInt Ref();
    tUInt Unref();
    tVoid Destroy();

private: // private methods
    tResult Search(vector<sPoint>& vecPoints, tUInt8 ui8LowerLimit, tUInt8 ui8UpperLimit, cv::Mat& matCannyImg, tFloat32 f32LineDivisor=0.5f);
    tResult TransmitAcceleration(tFloat32, tTimeStamp);
    tResult TransmitSteeringAngle(const tFloat32, tTimeStamp);
    tResult TransmitSteeringAnglePT1(const tFloat32, tTimeStamp);
    tResult TransmitHeadLights(const tBool, tTimeStamp);
    tResult LateralControl(vector<sPoint>& vecPoints, tTimeStamp tsInputTime);
    tResult LongitudinalControl(vector<sPoint>& vecPoints, tTimeStamp tsInputTime);
    tResult ProcessInput(IMediaSample* pSample, tTimeStamp);
    tResult ProcessFound();
    tResult ProcessOutput();
    tResult ReadProperties(const tChar* strPropertyName = NULL);
    tResult CreateAndTransmitGCL();

    tResult ProcessLines(vector<vector<sPoint> >& vecvecLines, vector<sPoint>& vecOut);
    tResult CorrectLanePoints(vector<sPoint>& vecLanePoints, sPoint& stMeanPoint, tInt& nMeanDiff);
    tResult MoveInvalidPoints(vector<sPoint>& vecLanePoints, tInt nMin, tInt nMax);
    tResult SwitchLeftRightIfNeeded(vector<sPoint>& vecLanePointsLeft, vector<sPoint>& vecLanePointsRight);
    tResult TransmitLaneTrackingInfo(const tFloat32, const tFloat32,const tFloat32,const tBool,tTimeStamp);
	tVoid ReInitialize();
	tVoid SaveImage(Mat originalImage , Mat nearImage , Mat farImage);

protected:
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr = NULL);

private:
    //f�r Search
    vector<sPoint> m_asAllpointsNear;
    tSize      m_szNearPointsCount;
    vector<sPoint> m_asAllpointsFar;
    tSize      m_szFarPointsCount;

    vector<vector<sPoint> > m_vecvecLines;

    vector<sPoint> m_vecOldLeft;
    vector<sPoint> m_vecOldMid;
    vector<sPoint> m_vecOldRight;
    tInt           m_nLastMeanLeft;
    tInt           m_nLastMeanMid;
    tInt           m_nLastMeanRight;


    // points and values for lane detection
    sPoint      m_sLaneCenterNear;
    tInt16      m_i16LaneWidth;
    sPoint      m_sPlaceToBe;
    tInt16      m_i16LaneWidthMinNear;
    tInt16      m_i16LaneWidthMaxNear;  
    tInt16      m_i16LaneWidthMinFar;
    tInt16      m_i16LaneWidthMaxFar;  

    tInt        m_nImageCutHeightNear;
    tInt        m_nNearLinesCount;

    // offset of the camera to the mid of the vehicle
    tFloat64    m_f64CamOffset;
    
    tInt        m_nCurrentNearLine;

    // PID-Controller values
    tFloat64    m_f64Kp;
    tFloat64    m_f64Ki;
    tFloat64    m_f64Kd;
    tInt        m_nCenterFromLeft;
    tInt        m_nCenterFromRight;
    tInt16      m_i16ErrorSum;
    tInt16      m_i16ErrorOld; 
    tFloat32    m_f32Ts;
    tInt16      m_i16FarLaneCenter;
    tInt16      m_i16FarLaneWidth;
    tFloat32    m_f32AccelerateOut;
    tInt16      m_i16FarLaneWidthCalc;

    // PT1-Controller values
    tFloat64    m_f64PT1Tau;
    tFloat64    m_f64PT1Sample;
    tFloat64    m_f64PT1Gain;
    tFloat32    m_f32PT1LastSteeringOut;
    tFloat32    m_f32PT1SteeringOut;
    tFloat64    m_f64PT1InputFactor;



    // active flag to enable driving
    tBool       m_bActive;
        
    // opencv members for line detection
    cv::Mat     m_matLine;    
    cv::Mat     m_matGrayNear;
    cv::Mat     m_matGrayFar;
    cv::Mat     m_matGrayThreshNear;
    cv::Mat     m_matGrayThreshFar;
    cv::Size    m_szCannySize;
    //cv::Size  m_szCannySizeFar;
    cv::Mat     m_matLineCannyNear;
    cv::Mat     m_matLineCannyFar;

    // DDL descriptions
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescBool;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescLaneTracking;

    // flag to check the first frame
    bool        m_bFirstFrame;
    // image counter 
    tUInt8      m_ui8Imagecount;
    // arduino timestamp value
    tUInt32     m_tsArduinoTime;
    // bitmap format of input pin
    tBitmapFormat m_sInputFormat;
        
    //Properties:
    
    tUInt8      m_ui8InitCtrl;
    tInt        m_nNearLine;
    tInt        m_nNearLineMaxOffset;
    tInt        m_nFarLine;
    tFloat32    m_f32AccelerationMax;
    tFloat32    m_f32AccelerationMin;
    tInt        m_nAccelerationFarNearDiff;
    tFloat32    m_f32SteeringAngle;
    tInt        m_nThresholdValue;
    tInt        m_nBlindCounter;
    tInt        m_nBlindCounterFar;
    tInt        m_nDriveTime;
    tInt        m_nEmergencyStopTime;
    
	tBool m_bHeadLights;
    tBool       m_bShowDebug;
    tBool       m_bEnableCannyWindows;
    tBool       m_bLightBeamTriggerEnabled;
        
    tHandle     m_hStopTimerNegative;
    tHandle     m_hStopTimerZero;
    tHandle     m_hEmergencyStopTimerNegative;
    tHandle     m_hEmergencyStopTimerZero;
    tHandle     m_hEmergencyStopTimerResume;
        
    // Critical Sections
    cCriticalSection    m_oTransmitSteerCritSection;
    cCriticalSection    m_oTransmitAccelCritSection;
    cCriticalSection    m_oTransmitLightCritSection;
    cCriticalSection    m_oRunCritSection;
    cCriticalSection    m_oTransmitLaneInfoCritSection;

    // members for signal registry
    typedef std::set<tSignalID> tActiveSignals;

    ucom::cObjectPtr<ISignalRegistryExtended> m_pISignalRegistry;
    cKernelMutex                              m_oLock;
    tActiveSignals                            m_oActive;

    // caching the error values for signal registry
    tFloat64            m_f64PT1ScaledError;
    tInt16              m_i16Error;
        
};

#endif 
