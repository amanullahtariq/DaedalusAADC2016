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

1) Filter for detecting parking area from the image

**********************************************************************
* $Author:: spiesra $  $Date:: 10-Feb-2016 08:29:07#$ $Rev:: 35003   $
* Updated By:: Amanullah Tariq 
**********************************************************************/

#ifndef _PARKINGSPOT_DETECTION_H_
#define _PARKINGSPOT_DETECTION_H_

#define OID_ADTF_PARKINGSPOT_DETECTION  "adtf.daedalus.parkingspot_detection"

class cParkingspotDetection : public adtf::cFilter {

  ADTF_FILTER(OID_ADTF_PARKINGSPOT_DETECTION, "DAEDALUS ParkingspotDetection ", adtf::OBJCAT_DataFilter);
protected:
   cVideoPin VideoInputPin;
   cInputPin DriverActionPin;
   cInputPin StartPin;
   cOutputPin ParkingInfoPin;
private:
	bool isFirstFrame;
	tBitmapFormat m_sInputFormat; 
	tInt32 DriverNextAction;
	tBool isEnable;
	Mat inputImage;

	//FLAG
	tBool DriverModuleBufferReaderFlag;
	tBool m_bShowLog;
	tFloat32 distFactor;

	//driver buffer
    tBufferID DriverNextActionBuffer; 
    tBufferID DriverTimeBuffer; 
    tBufferID DriverIsProcessedBuffer;

private:
	tResult ProcessImage(IMediaSample* sample, tTimeStamp tsInputTime);

    // DDL descriptions
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescBool;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionDecisionInput;

	// Critical Sections
    cCriticalSection m_oParkingInfoCritSection;

	// arduino timestamp value
    tUInt32     m_tsArduinoTime;

 public:
    cParkingspotDetection(const tChar* __info);
    virtual ~cParkingspotDetection();

protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);
	/*! creates all the input Pins*/
	tResult CreateInputPins(__exception = NULL);
	/*! creates all the output Pins*/
	tResult CreateOutputPins(__exception = NULL);
	tVoid ReadProperties();
	tVoid GetParkingInfo(Mat inputImage);
	tVoid GenerateIPM(Mat inputImg, Mat &outputImg);
	

	tResult TransmitParkingInfo(tFloat32 value, tTimeStamp tsInputTime);

};

//*************************************************************************************************
#endif // _PARKINGSPOT_DETECTION_H_
