#ifndef _DAEDALUS_MAPPING_FILTER_H_
#define _DAEDALUS_MAPPING_FILTER_H_


#define OID_ADTF_DAEDALUS_MAPPING "adtf.daedalus.mapping_filter"

class Map;

//*************************************************************************************************
class cDaedalusMapping : public adtf::cFilter
{
    ADTF_FILTER(OID_ADTF_DAEDALUS_MAPPING, "DAEDALUS Mapping Filter", adtf::OBJCAT_DataFilter);

struct Measurement
{
	double	value;
	Vec3d	position;
};

protected:
    cVideoPin		m_oRGBVideo;		// output pin for the map
    cVideoPin		m_oVideoInputPin;	// input from the camera
	
	cInputPin		m_inputPinSteeringAngle;
    cInputPin		m_inputPinDeltaDistance;

    cOutputPin		m_DecisionOutput;

    cInputPin		m_usSensorFrontRight;

    cInputPin		m_inputPinStart;

    //cObjectPtr<IMediaTypeDescription> m_pDescriptionDecisionOutput;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalValue;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescBoolSignalValue;
	

public:
    cDaedalusMapping(const tChar* __info);
    virtual ~cDaedalusMapping();

protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);
private:
	tResult ProcessInput(IMediaSample* pSample, tTimeStamp);
	tResult	TransmitToDecision();
	//tVoid ApplyClahe(Mat inputImage, Mat &outputImage,int limit);
					   
private:
	tInt        m_nWidthVideo;                /**< width of the stream */
    tInt        m_nHeightVideo;                /**< height of the stream */

	tBitmapFormat    m_sBitmapFormat;	// Bitmap format for video output
	
	// flag to check the first frame
    bool        m_bFirstFrame;
	// bitmap format of input pin
    tBitmapFormat m_sInputFormat;
	
	tUInt8      m_ui8InitCtrl;
	
	double		m_fSteeringAngle;
	
	
	Map*		m_cMap;
	
	// Map parameters
	double		m_fAxisDistance;		// Distance of front and rear axis in m	
	double		m_dMtoP;				// Conversion factor of m to px																				
	Vec3d		m_vCar;					// Car's position relative to received image
	double		m_fMaxDistance;			// maximum distance for landmark association
	double		m_fCameraOffset;

	// IPM
	// The 4-points at the input image	
	vector<Point2f> m_vOrigPoints;
	// The 4-points correspondences in the destination image
	vector<Point2f> m_vDstPoints;
	IPM*		m_pIPM;
	int 		m_iHorizon;

	int 		m_iThreshold;
	double 		m_dBlobBound;
	int 		m_iBlobSupport;

	double		m_dDistanceAcc;
	bool		m_bDoCorrectionStep;

	pthread_mutex_t m_lMapLock;
	pthread_mutex_t m_lSensorLock;

//	vector<double>	m_vBins;
//	vector<int>		m_vMeasureHits;
	Landmark* 	m_pLastHLine;

	double		m_dSensorAcc;
	int 		m_iSensorCtr;
	double 		m_dTravelledDist;
	vector<Measurement>	m_vMeasurements;

	bool 		m_bIsActive;
	bool 		m_bStart;
	
};

//*************************************************************************************************
#endif // _DAEDALUS_MAPPING_FILTER_H_
