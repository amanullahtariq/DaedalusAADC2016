
#ifndef _FILTER_DRIVE_H_
#define _FILTER_DRIVE_H_

#define OID_ADTF_FILTER_DRIVE "daedalus.aadc.filter_drive"

class FilterDrive : public adtf::cTimeTriggeredFilter {
  ADTF_FILTER(OID_ADTF_FILTER_DRIVE, "Drive Filter", adtf::OBJCAT_DataFilter);
  public:
    FilterDrive(const tChar* __info);
    virtual ~FilterDrive();
    tResult Cycle(__exception=NULL);

  private:
    // Filter methods
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);
    
    // Output pin
    cOutputPin data_;

    // Coder Descriptor for the input pins
    cObjectPtr<IMediaTypeDescription> output_stream_description_;

    int counter_;
    bool debug_;
};

#endif // _FILTER_DRIVE_H_
