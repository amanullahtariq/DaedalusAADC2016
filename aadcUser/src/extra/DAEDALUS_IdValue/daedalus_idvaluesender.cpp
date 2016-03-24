/**
 *
 * Advanced example to show 
 *     - how the ExtendedData interface will be used.
 *     - how a filter can react on EDS events.
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: voigtlpi $
 * $Date: 2014-07-15 16:21:05 +0200 (Di, 15 Jul 2014) $
 * $Revision: 49172 $
 *
 * @remarks
 *
 */

#include "stdafx.h"
#include "daedalus_idvaluesender.h"
// -------------------------------------------------------------------------------------------------
ADTF_FILTER_PLUGIN("DAEDALUS ID Value Sender", OID_DAEDALUS_ID_VALUE_SENDER, cIdValueSender);
// -------------------------------------------------------------------------------------------------


#define PROP_ACTION "Action Number"
#define PROP_SHOW_LOG "Show Log"
#define PROP_START_DELAY "Start Delay"
#define PROP_INTERVAL_MS "Interval MS"
#define PROP_ENABLE "Enable"
// -------------------------------------------------------------------------------------------------
cIdValueSender::cIdValueSender(const tChar* __info) : adtf::cTimeTriggeredFilter(__info) {
// -------------------------------------------------------------------------------------------------
  SetPropertyInt(PROP_ACTION, 1);
  SetPropertyInt(PROP_START_DELAY, 10);
  SetPropertyInt(PROP_INTERVAL_MS, 100);
  SetPropertyBool(PROP_SHOW_LOG, true);
  SetPropertyBool(PROP_ENABLE, true);
}

// -------------------------------------------------------------------------------------------------
cIdValueSender::~cIdValueSender() {
// -------------------------------------------------------------------------------------------------

}

// -------------------------------------------------------------------------------------------------
tResult cIdValueSender::Init(tInitStage eStage, __exception) {
// -------------------------------------------------------------------------------------------------
  // never miss calling the parent implementation!!
  RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
  
  // Initialize the descriptors and media type stuff here
  cObjectPtr<IMediaDescriptionManager> description_manager;
  RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,
    IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&description_manager,__exception_ptr));

  if (eStage == StageFirst) {
    // Create and register the output pin
    tChar const * stream_description = description_manager->GetMediaDescription("tIdValue");
    RETURN_IF_POINTER_NULL(stream_description);        
    cObjectPtr<IMediaType> output_type = new cMediaType(0, 0, 0, "tIdValue",
      stream_description,IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
		RETURN_IF_FAILED(output_type->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION,
		  (tVoid**)&output_stream_description_));

    RETURN_IF_FAILED(data_.Create("Action", output_type, NULL));
		RETURN_IF_FAILED(RegisterPin(&data_));


        tChar const * strDescSignalValue = description_manager->GetMediaDescription("tSignalValue");	
        RETURN_IF_POINTER_NULL(strDescSignalValue);	
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);    	
  
		 //get mediatype description for output data type
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutput));

        //create pin for speed output data
        RETURN_IF_FAILED(m_oOutputValue.Create("Signal_Value", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oOutputValue));


		tChar const * strDescignalValue = description_manager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescignalValue);        
        cObjectPtr<IMediaType> pTypeBoolValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
        RETURN_IF_FAILED(pTypeBoolValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool)); 

        //create pin for output
		RETURN_IF_FAILED(EnablePin.Create("Enable",pTypeBoolValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&EnablePin));
  }
  
  else if (eStage == StageNormal) {
    // In this stage you would do further initialisation and/or create your dynamic pins.
    // Please take a look at the demo_dynamicpin example for further reference.
  }
  
  else if (eStage == StageGraphReady) {
    // All pin connections have been established in this stage so you can query your pins
    // about their media types and additional meta data.

		this->SetInterval(GetPropertyInt(PROP_INTERVAL_MS) * 1000);
		
		counter_ = 0;
		debug_ = GetPropertyBool(PROP_SHOW_LOG);
		enable = GetPropertyBool(PROP_ENABLE);
		DataSend = false;
  }
  
  RETURN_NOERROR;
}

// -------------------------------------------------------------------------------------------------
tResult cIdValueSender::Shutdown(tInitStage eStage, __exception) {
// -------------------------------------------------------------------------------------------------
  // Call the base class implementation
  return cFilter::Shutdown(eStage, __exception_ptr);
}

// -------------------------------------------------------------------------------------------------
tResult cIdValueSender::Cycle(__exception) {
// -------------------------------------------------------------------------------------------------
	if (counter_ < GetPropertyInt(PROP_START_DELAY) * (1000.0f / GetPropertyInt(PROP_INTERVAL_MS)))
	{
      counter_++;
      RETURN_NOERROR;
    }
	else if (!DataSend)
	{
		cObjectPtr<IMediaSample> pMediaSample;
		AllocMediaSample((tVoid**)&pMediaSample);

		//allocate memory with the size given by the descriptor
		cObjectPtr<IMediaSerializer> pSerializer;
		output_stream_description_->GetMediaSampleSerializer(&pSerializer);
		tInt nSize = pSerializer->GetDeserializedSize();
		pMediaSample->AllocBuffer(nSize);

		tInt32 value = GetPropertyInt(PROP_ACTION);
		tUInt32 timeStamp = 0;
		tBool isExecuted = tFalse;
    
		{
		  __adtf_sample_write_lock_mediadescription(output_stream_description_, pMediaSample, coder);
		  coder->Set("i32Value", (tVoid*) &value);
		  coder->Set("ui32ArduinoTimestamp", (tVoid*) &timeStamp);
		  coder->Set("bIsExecuted", (tVoid*) &isExecuted);
		}

		//transmit media sample over output pin
		pMediaSample->SetTime(_clock->GetStreamTime());
		if (debug_) LOG_INFO(cString::Format("ID Value: Action value: %d", value));
		data_.Transmit(pMediaSample);
		DataSend = true;
	}
	
	TransmitSignalValue();
	TransmitEnableValue();
    RETURN_NOERROR;
}

tResult cIdValueSender::TransmitEnableValue()
{
	tBool send = tTrue;
   __synchronized_obj(m_oBoolSenderCritSection);
    
    //create new media sample
    cObjectPtr<IMediaSample> pSampleAccel;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSampleAccel));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBool->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pSampleAccel->AllocBuffer(nSize));
        
    {
          __adtf_sample_write_lock_mediadescription(m_pDescriptionBool, pSampleAccel, pCoderOutput);
 
          pCoderOutput->Set("bValue", (tVoid*)&send);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);    
    }
          
     pSampleAccel->SetTime(_clock->GetStreamTime());
	 RETURN_IF_FAILED(EnablePin.Transmit(pSampleAccel));

     RETURN_NOERROR;
}



tResult cIdValueSender::TransmitSignalValue()
{
	tFloat32 value = rand() % 10 + 1; 
	tUInt32 m_tsArduinoTime = 0;
    //create new media sample
    cObjectPtr<IMediaSample> pSampleAccel;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSampleAccel));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pSampleAccel->AllocBuffer(nSize));
        
    {
          __adtf_sample_write_lock_mediadescription(m_pDescriptionOutput, pSampleAccel, pCoderOutput);
 
          pCoderOutput->Set("f32Value", (tVoid*)&value);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
          
     pSampleAccel->SetTime(_clock->GetStreamTime());
     RETURN_IF_FAILED(m_oOutputValue.Transmit(pSampleAccel));

	RETURN_NOERROR;
}
