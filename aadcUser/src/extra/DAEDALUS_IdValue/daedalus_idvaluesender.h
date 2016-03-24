/**
 *
 * Advanced example to show 
 *     - how the ExtendedData interface will be used.
 *     - how a filter can react on EDS events.
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: A1RHEND $
 * $Date: 2013-04-11 17:25:32 +0200 (Do, 11 Apr 2013) $
 * $Revision: 37969 $
 *
 * @remarks
 *
 */
#ifndef _DEMO_EXTENDED_DATA_FILTER_HEADER_
#define _DEMO_EXTENDED_DATA_FILTER_HEADER_

#define OID_DAEDALUS_ID_VALUE_SENDER "daedalus.dev.id_value_sender"

class cIdValueSender : public adtf::cTimeTriggeredFilter
{
  ADTF_FILTER(OID_DAEDALUS_ID_VALUE_SENDER, "DAEDALUS ID Value Sender", adtf::OBJCAT_DataFilter);
  public:
    cIdValueSender(const tChar* __info);
    virtual ~cIdValueSender();
   tResult Cycle(__exception=NULL);

	 //tResult OnPinEvent(IPin* pSource,
  //                     tInt nEventCode,
  //                     tInt nParam1,
  //                     tInt nParam2,
  //                     IMediaSample* pMediaSample);

  private:
    // Filter methods
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);
    
    // Input pin
    cOutputPin data_;
	cOutputPin m_oOutputValue;
	cOutputPin EnablePin;
    // Coder Descriptor for the input pins
     cObjectPtr<IMediaTypeDescription> output_stream_description_;
	 cObjectPtr<IMediaTypeDescription> m_pDescriptionOutput; 
	 cObjectPtr<IMediaTypeDescription> m_pDescriptionBool;
	 tResult TransmitSignalValue();
	 tResult TransmitEnableValue();

    int counter_;
    bool debug_;
	bool enable;
	bool DataSend;
	tUInt32 timeStamp;

	 cCriticalSection    m_oBoolSenderCritSection;
	tBufferID m_szIDBoolValueOutput;    
	tBool m_bIDsBoolValueOutput;
	tBufferID m_szIDArduinoTimestampOutput; 
};



#endif // _DEMO_EXTENDED_DATA_FILTER_HEADER_
