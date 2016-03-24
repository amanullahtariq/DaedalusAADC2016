#include "stdafx.h"
#include "DAEDALUS_Mapping.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("DAEDALUS Mapping Filter", OID_ADTF_DAEDALUS_MAPPING, cDaedalusMapping);


cDaedalusMapping::cDaedalusMapping(const tChar* __info):cFilter(__info)
{
	// TODO inroduce filter properties for the following parameters
/*	
	SetPropertyFloat("Achsis Distance", 0.35);
	SetPropertyFloat("Meter to Pixel", 180);
	SetPropertyFloat("Max matching dist", 10);
	SetPropertyFloat("Camera offset", 0.25);
*/
	SetPropertyInt("Threshold", 75);

	m_nWidthVideo = 640;
	m_nHeightVideo = 480;
	m_fAxisDistance = 0.35;
	m_dMtoP = 800;			// 410
	m_vCar = Vec3d(	320, // 320
			480 + (0.42*m_dMtoP), // 0,42*m_dMtoP
			0	);
	m_fMaxDistance = 150.;
	m_iHorizon = 260;
	m_iThreshold = 75;
	m_dBlobBound = 10.0; // 60
	m_iBlobSupport = 20; // 140

}

cDaedalusMapping::~cDaedalusMapping()
{
}

// tVoid cDaedalusMapping::ApplyClahe(Mat inputImage, Mat &outputImage,int limit)
// {
// 	Ptr<CLAHE> clahe = createCLAHE();
// 	clahe->setClipLimit(limit);
// 	clahe->apply(inputImage, outputImage);
// }

tResult cDaedalusMapping::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    
    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
		// Initialize the description manager
    	cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
    
    	// Get the media description for tSignalValue
    	tChar const* strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    	RETURN_IF_POINTER_NULL(strDescSignalValue);

		// get media type for tSignalValue
		cObjectPtr<IMediaType> pMediaTypeSignalValue = new cMediaType(0,0,0,"tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pMediaTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalValue));
		
    	// Get the media description for tBoolSignalValue
    	tChar const* strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    	RETURN_IF_POINTER_NULL(strDescBoolSignalValue);

  		// get media type for tBoolSignalValue
		cObjectPtr<IMediaType> pMediaTypeBoolSignalValue = new cMediaType(0,0,0,"tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pMediaTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescBoolSignalValue));


		// create and register m_inputPinSteeringAngle
		RETURN_IF_FAILED(m_inputPinSteeringAngle.Create("Steering_Angle", pMediaTypeSignalValue, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_inputPinSteeringAngle));
		
		// create and register m_inputPinDeltaDistance
		RETURN_IF_FAILED(m_inputPinDeltaDistance.Create("Delta_Distance", pMediaTypeSignalValue, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_inputPinDeltaDistance));
		
        // Video Input
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));
		
		// Video output
        RETURN_IF_FAILED(m_oRGBVideo.Create("Video_RGB", adtf::IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oRGBVideo));

        // Initialize input pin Ultrasound front right 
        RETURN_IF_FAILED(m_usSensorFrontRight.Create("UltraSound_Front_Right", pMediaTypeSignalValue, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_usSensorFrontRight));

        //RETURN_IF_FAILED(pMediaTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionDecisionOutput));    
        // Decision output
        RETURN_IF_FAILED(m_DecisionOutput.Create("Decision_output",pMediaTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_DecisionOutput));

        // Start pin
        RETURN_IF_FAILED(m_inputPinStart.Create("Start", pMediaTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_inputPinStart));
    }
    else if (eStage == StageNormal)
    {

		/*m_fAxisDistance = static_cast<double> (GetPropertyFloat("Achsis Distance"));
		m_dMtoP = static_cast<double> (GetPropertyFloat("Meter to Pixel"));
		m_fMaxDistance = static_cast<double> (GetPropertyFloat("Max matching dist"));
		m_vCar = Vec3d(	m_nWidthVideo / 2,
			m_nHeightVideo + m_fCameraOffset * m_dMtoP,
			0	);*/
		m_iThreshold = static_cast<int> (GetPropertyFloat("Threshold"));

        //set the videoformat of the rgb video pin
        m_sBitmapFormat.nWidth = m_nWidthVideo;
        m_sBitmapFormat.nHeight = m_nHeightVideo;
        m_sBitmapFormat.nBitsPerPixel = 24;
        m_sBitmapFormat.nPixelFormat = cImage::PF_RGB_888;
        m_sBitmapFormat.nBytesPerLine = m_nWidthVideo * 3;
        m_sBitmapFormat.nSize = m_sBitmapFormat.nBytesPerLine * m_nHeightVideo;
        m_sBitmapFormat.nPaletteSize = 0;
        m_oRGBVideo.SetFormat(&m_sBitmapFormat, NULL);
		
		// initialize map
		m_cMap = new Map(Vec2d(1000, 1000), m_vCar, m_dMtoP, m_fAxisDistance, m_fMaxDistance);

		m_bFirstFrame = true;
		m_bIsActive = false;
		m_bStart = false;

		m_dSensorAcc = 0;
		m_iSensorCtr = 0;
		m_dTravelledDist = 0;
		m_vMeasurements.clear();

		m_dDistanceAcc = 0;
		m_bDoCorrectionStep = true;

		//m_vBins.clear();
		//m_vMeasureHits.clear();
		m_pLastHLine = NULL;


		pthread_mutex_init(&m_lMapLock, NULL);
		pthread_mutex_init(&m_lSensorLock, NULL);
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult cDaedalusMapping::TransmitToDecision()
{
	cObjectPtr<IMediaSample> pMediaSample;
   RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

   cObjectPtr<IMediaSerializer> pSerializer;
   m_pCoderDescSignalValue->GetMediaSampleSerializer(&pSerializer);
   tInt nSize = pSerializer->GetDeserializedSize();

   tFloat32 timestamp = 0;
   tFloat32 value = 1;

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {   // scope for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalValue, pMediaSample, pCoder);

		pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        pCoder->Set("f32Value", (tVoid*)&value);
    } 

    pMediaSample->SetTime(_clock->GetStreamTime());
    m_DecisionOutput.Transmit(pMediaSample);
   
   RETURN_NOERROR;
}

tResult cDaedalusMapping::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception: 
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.
    
    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
    	   	delete(m_cMap);
		   	//delete(m_pIPM);
		   	m_pLastHLine = NULL;
		   	pthread_mutex_unlock(&m_lSensorLock);
		   	pthread_mutex_unlock(&m_lMapLock);
		   	pthread_mutex_destroy(&m_lSensorLock);
		   	pthread_mutex_destroy(&m_lMapLock);
    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cDaedalusMapping::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);
		
		tTimeStamp InputTimeStamp;
        InputTimeStamp = pMediaSample->GetTime();


        if(pSource == &m_oVideoInputPin && m_bIsActive)
        {
            //Videoformat
            if (m_bFirstFrame)
            {        
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));
                cObjectPtr<IMediaTypeVideo> pTypeVideo;
                RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
                const tBitmapFormat* pFormat = pTypeVideo->GetFormat();                                
                if (pFormat == NULL)
                {
                    LOG_ERROR("No Bitmap information found on pin \"input\"");
                    RETURN_ERROR(ERR_NOT_SUPPORTED);
                }
                m_sInputFormat.nPixelFormat = pFormat->nPixelFormat;
                m_sInputFormat.nWidth = pFormat->nWidth;
                m_sInputFormat.nHeight =  pFormat->nHeight;
                m_sInputFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
                m_sInputFormat.nBytesPerLine = pFormat->nBytesPerLine;
                m_sInputFormat.nSize = pFormat->nSize;
                m_sInputFormat.nPaletteSize = pFormat->nPaletteSize;
                m_bFirstFrame = false;
				
				// IPM transformation points Nice experimental
				int shift = 30;  // 0
				int hor_dist = 200; // 160
				int bot_dist = 170; // 100
				m_vOrigPoints.push_back( Point2f(m_sInputFormat.nWidth/2 + shift, m_sInputFormat.nHeight - 90) );
				m_vOrigPoints.push_back( Point2f(m_sInputFormat.nWidth + bot_dist + shift, m_sInputFormat.nHeight - 90) );
				m_vOrigPoints.push_back( Point2f(m_sInputFormat.nWidth/2 + hor_dist + shift, m_iHorizon) );
				m_vOrigPoints.push_back( Point2f(m_sInputFormat.nWidth/2 + shift, m_iHorizon) );

				m_vDstPoints.push_back( Point2f(0, m_sInputFormat.nHeight) );
				m_vDstPoints.push_back( Point2f(m_sInputFormat.nWidth, m_sInputFormat.nHeight) );
				m_vDstPoints.push_back( Point2f(m_sInputFormat.nWidth, 0) );
				m_vDstPoints.push_back( Point2f(0, 0) );

				// IPM object
				m_pIPM = new IPM(	Size(m_sInputFormat.nWidth, m_sInputFormat.nHeight),
									Size(m_sInputFormat.nWidth, m_sInputFormat.nHeight),
									m_vOrigPoints,
									m_vDstPoints );
LOG_ERROR(cString::Format("mtop: %f", m_dMtoP));
LOG_ERROR(cString::Format("max dist: %f", m_fMaxDistance));
LOG_ERROR(cString::Format("Achsis dist: %f", m_fAxisDistance));
LOG_ERROR(cString::Format("Car x: %f, y: %f", m_vCar[0], m_vCar[1]));
            }
            	ProcessInput(pMediaSample, InputTimeStamp);
            //if (m_bDoCorrectionStep){
            //}
        }
		
		// if a new steering angle is received, update the internal value
        if (pSource == &m_inputPinSteeringAngle && m_bIsActive)
        {
				cObjectPtr<IMediaCoder> pCoder;
				RETURN_IF_FAILED(m_pCoderDescSignalValue->Lock(pMediaSample, (IMediaCoder**)&pCoder));

				tFloat32 _steeringAngle;
				pCoder->Get("f32Value", &_steeringAngle);

				m_pCoderDescSignalValue->Unlock(pCoder);
				
				if(m_fSteeringAngle != _steeringAngle)
				{
					m_fSteeringAngle = 90;

					// Kalman Update step	
					//pthread_mutex_lock(&m_lMapLock);
					//m_cMap->updateStep( m_fSteeringAngle, m_dDistanceAcc );
					//pthread_mutex_unlock(&m_lMapLock);
					//m_dDistanceAcc = 0;
					//m_fSteeringAngle = _steeringAngle;
					//m_bDoCorrectionStep = true;
// LOG_ERROR(cString::Format("New Steering angle: %f", m_fSteeringAngle));	
				}
        }
		// if a new travelled distance is received, update the map according to the
		// current steering angle and the travelled distance
		else if (pSource == &m_inputPinDeltaDistance && m_bIsActive)
        {
			cObjectPtr<IMediaCoder> pCoder;
			RETURN_IF_FAILED(m_pCoderDescSignalValue->Lock(pMediaSample, (IMediaCoder**)&pCoder));
		
			tFloat32 _distance;
			pCoder->Get("f32Value", &_distance);
		
			m_pCoderDescSignalValue->Unlock(pCoder);

			m_dDistanceAcc += _distance;
			
			if (NULL != m_pLastHLine)
			{
				m_dTravelledDist += _distance;
			}

			if (m_dDistanceAcc > 0.01)
			{	
					// Kalman Update step
					pthread_mutex_lock(&m_lMapLock);
					m_cMap->updateStep( m_fSteeringAngle, m_dDistanceAcc );
					pthread_mutex_unlock(&m_lMapLock);
					m_dDistanceAcc = 0;
					m_bDoCorrectionStep = true;
			}
        }
        else if(pSource == &m_usSensorFrontRight && m_bIsActive)
        {
	       	if (NULL != m_pLastHLine)
	       	{
	        	cObjectPtr<IMediaCoder> pCoder;
				RETURN_IF_FAILED(m_pCoderDescSignalValue->Lock(pMediaSample, (IMediaCoder**)&pCoder));

				tFloat32 _sensor_value = 0.0;
				tUInt32 _timestamp = 0;

				pCoder->Get("ui32ArduinoTimestamp", &_timestamp);
				pCoder->Get("f32Value", &_sensor_value);
		       	
		       	m_pCoderDescSignalValue->Unlock(pCoder);


		       	if (_sensor_value < 300 && _sensor_value > 0.01)
		       	{
		       	pthread_mutex_lock(&m_lSensorLock);

	       		/*int bin = static_cast<int>(m_dTravelledDist / 0.02);
	       		if (static_cast<int>(m_vBins.size()) - 1 < bin)
	       		{
	       			m_vBins.push_back(_sensor_value);
	       			m_vMeasureHits.push_back(1);
	       		}
	       		else
	       		{
	       			m_vBins.at(bin) += _sensor_value;
	       			m_vMeasureHits.at(bin) += 1;
	       		}*/

	       		m_dSensorAcc = ((m_dSensorAcc * m_iSensorCtr) + _sensor_value) / (m_iSensorCtr + 1);
	       		m_iSensorCtr++;

	       		Measurement m;
	       		m.value = _sensor_value;
	       		m.position = Vec3d(320 + (_sensor_value * 10), m_dTravelledDist * m_dMtoP, 1);
	       		m_vMeasurements.push_back(m);
//LOG_ERROR(cString::Format("Bin count: %d", static_cast<int>(m_vBins.size())));


		       	pthread_mutex_unlock(&m_lSensorLock);
		       }

	       	}
        }

        else if(pSource == &m_inputPinStart)
        {
        	cObjectPtr<IMediaCoder> pCoder;
			RETURN_IF_FAILED(m_pCoderDescBoolSignalValue->Lock(pMediaSample, (IMediaCoder**)&pCoder));

			tBool _start = false;
			tUInt32 _timestamp = 0;

			pCoder->Get("ui32ArduinoTimestamp", &_timestamp);
			pCoder->Get("bValue", &_start);
	       	
	       	m_pCoderDescSignalValue->Unlock(pCoder);

	       	// Deinitialize everything	
	        m_dDistanceAcc = 0;       	
//			m_vBins.clear();
//			m_vMeasureHits.clear();
//			m_dTravelledDist = 0;

			m_dSensorAcc = 0;
			m_iSensorCtr = 0;
			m_pLastHLine = NULL;
			m_cMap->clear();

     		m_bIsActive = _start;
LOG_ERROR(cString::Format("MAPPING received %d from Decision", m_bIsActive));

        }
    }

    RETURN_NOERROR;
}

tResult cDaedalusMapping::ProcessInput(IMediaSample* pSample, tTimeStamp tsInputTime)
{
    if(m_ui8InitCtrl < -1)         // This loop is necessary to boot the car's controller (needs neutral signal for a certain time)
    {
        m_ui8InitCtrl++;
        m_bDoCorrectionStep = true;
    }
    else
    {
        // VideoInput
        RETURN_IF_POINTER_NULL(pSample);

        m_bDoCorrectionStep = false;

        const tVoid* l_pSrcBuffer;
    
        IplImage* oImg = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
        RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
        oImg->imageData = (char*)l_pSrcBuffer;
        Mat image(cvarrToMat(oImg));
        cvReleaseImage(&oImg);
        pSample->Unlock(l_pSrcBuffer);
				
		Mat gray, binary, ipm;
	    
		// Image processing
	    cvtColor( image, gray, CV_BGR2GRAY );							// RGB to Grayscale
	    //GaussianBlur( gray, gray, Size( 5, 5 ), 0, 0 );					// Gaussian blurring (maybe superfluous)
		equalizeHist(gray, gray);
	    gray = DAEDALUS::correctGamma( gray, 0.2 );						// Gamma correction (Contrast enhancement)


		// Apply IPM
		RETURN_IF_POINTER_NULL(m_pIPM);
		m_pIPM->applyHomography( gray, ipm );

	    threshold( ipm, binary, m_iThreshold, 255.0, THRESH_BINARY );	// Grayscale to Binary (Thresholding)



		std::vector<Landmark*> _map;

		// Measurment step
	    std::vector<DAEDALUS::Blob> _blobs = DAEDALUS::detectBlobs(binary, m_dBlobBound);


	    // Filtering Landmarks
		std::vector<DAEDALUS::Blob> _blobs_temp;
		for ( int i = 0; i < static_cast<int>(_blobs.size()); i++ )
		{
			if ( _blobs.at(i).support > m_iBlobSupport && _blobs.at(i).pt[0] > 290 )
			{	
				_blobs_temp.push_back(_blobs.at(i));
			}
		}
		

		// Detecting horizontal lines
	    std::vector<DAEDALUS::Blob> _hLines = DAEDALUS::detectHorLine(_blobs_temp, 6, 5); // 15, 5

	    // Reject if too many noise
	    if (_hLines.size() > 10)
	    {
	    	_hLines.clear();
	    }


		// Adapting Threshold
/*	    if (_blobs.size() > 200)  // 160
		{
	      m_iThreshold *= 1.1;
		  m_iThreshold = m_iThreshold > 255 ? 255 : m_iThreshold;

			m_bDoCorrectionStep = true;
//LOG_ERROR(cString::Format("Thresh inc: %d", m_iThreshold));
		}
	    else if (_blobs.size() < 100) // 160
	    {
	    	m_iThreshold *= 0.95;
		  m_bDoCorrectionStep = true;
//LOG_ERROR(cString::Format("Thresh dec: %d", m_iThreshold));
		}
		else*/ if(!m_bStart)
		{
			m_bStart = true;
LOG_ERROR(cString::Format("READY"));
		}
		
		if (m_bStart)
		{
			
			for ( int i = 0; i < static_cast<int>(_hLines.size()); i++ )
			{
				_hLines.at(i).pt[0] = 320;		// centering each horizontal line
			}
			
			// Kalman correction
			pthread_mutex_lock(&m_lMapLock);
			m_cMap->correctionStep(_hLines);	// _blobs
			pthread_mutex_unlock(&m_lMapLock);
			
			_map = m_cMap->getMap();


			// Find the last passed horizontal line
			double _dMin = DBL_MAX;
			Landmark* _lMin = NULL;
	        for ( int i = 0; i < static_cast<int>(_map.size()); i++ )
			{
				double _d = _map.at(i)->mean[1] - m_vCar[1];
				if (_d >= 0 && _d < _dMin)
				{
					_dMin = _d;
					_lMin = _map.at(i);

				}
			}

			if (_lMin != NULL && m_pLastHLine != _lMin)
			{
LOG_ERROR(cString::Format("Passing HLine, age: %d, fitness: %f", _lMin->age, _lMin->fitness));
	       		pthread_mutex_lock(&m_lSensorLock);

				if (NULL != m_pLastHLine)
				{
					double _parkingDistance = abs(m_pLastHLine->mean[1] - _lMin->mean[1]);
					//double avg = 0;
					//for (int i = 0; i < static_cast<int>(m_vBins.size()); i++)
					//{
					//	avg += m_vBins.at(i) / m_vMeasureHits.at(i);
					//}
					//avg = avg / static_cast<int>(m_vBins.size());

LOG_ERROR(cString::Format("Obj. Dist: %f", m_dSensorAcc));
LOG_ERROR(cString::Format("Parking Dist: %f", _parkingDistance));

					double  _sensorThresh = _parkingDistance < 300 ? 0.45 : 0.45;

					if (_parkingDistance >= 0.25 * m_dMtoP && m_dSensorAcc > _sensorThresh)
					{
						TransmitToDecision();
						m_bIsActive = false;
						//m_bStart = false;
LOG_ERROR(cString::Format("Mapping: Start Parking"));
					}
				}
				//m_vBins.clear();
				//m_vMeasureHits.clear();
				m_dTravelledDist = 0;
				m_pLastHLine = _lMin;
				m_dSensorAcc = 0;
				m_iSensorCtr = 0;
				m_vMeasurements.clear();


	       		pthread_mutex_unlock(&m_lSensorLock);
			}

		}

/*
		// generate output image of the map
		Mat out; //  = Mat::zeros(m_nHeightVideo, m_nWidthVideo, CV_8UC3);
		cvtColor( gray, out, CV_GRAY2RGB ); 
		int scale = 3;
		
		for ( int i = 0; i < static_cast<int>(_blobs_temp.size()); i++ )
		{
			circle(	out, 
					Point2d(_blobs_temp.at(i).pt[0] / scale, _blobs_temp.at(i).pt[1] / scale),
					2,
					Scalar(255, 0, 0),
					2 );
		}


		_map = m_cMap->getMap();
		for ( int i = 0; i < static_cast<int>(_map.size()); i++ )
		{
//LOG_ERROR(cString::Format("Age: %d", _map.at(i)->age));
			int red = _map.at(i)->age * 5;
			red = red > 255 ? 255 : red;
			circle(	out, 
					Point2d(_map.at(i)->mean[0] / scale, _map.at(i)->mean[1] / scale),
					2,
					Scalar(red, 0, 255),
					2 );
		}


		for ( int i = 0; i < static_cast<int>(_hLines.size()); i++ )
		{
			int red = _hLines.at(i).support / 80;
			red = red > 255 ? 255 : red;
			circle(	out, 
					Point2d(200 / scale, _hLines.at(i).pt[1] / scale),
					2,
					Scalar(0, 255, red),
					2 );
		}

		if (m_pLastHLine != NULL)
		{		
			for(int i = 0; i < static_cast<int>(m_vMeasurements.size()); i++)
			{
				circle(	out, 
						Point2d(	m_vMeasurements.at(i).position[0]  / scale, 
									(m_pLastHLine->mean[1] - m_vMeasurements.at(i).position[1]) / scale),
						2,
						Scalar(0, 255, 255),
						2 );	
			}
		}

		circle(	out, 
					Point2d(m_vCar[0] / scale, m_vCar[1] / scale),
					2,
					Scalar(255, 255, 0),
					2 );
        
		//m_pIPM->drawPoints(m_vOrigPoints, out);

		// transmit data in media sample over the output pin
        cObjectPtr<IMediaSample> pNewRGBSample;
        if (IS_OK(AllocMediaSample(&pNewRGBSample)))
        {
			size_t imSize = out.total() * out.elemSize();
            tTimeStamp tmStreamTime = adtf_util::cHighResTimer::GetTime();
            pNewRGBSample->Update(tmStreamTime, out.data, tInt32(imSize), 0);
            RETURN_IF_FAILED(m_oRGBVideo.Transmit(pNewRGBSample));
        }
*/
    }

    RETURN_NOERROR;            
}
