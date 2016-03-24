#include "DAEDALUS_util.h"

cv::Mat DAEDALUS::correctGamma( cv::Mat& img, double gamma ) {
	double inverse_gamma = 1.0 / gamma;

	cv::Mat lut_matrix(1, 256, CV_8UC1 );
	uchar * ptr = lut_matrix.ptr();
	for( int i = 0; i < 256; i++ )
	ptr[i] = (int)( pow( (double) i / 255.0, inverse_gamma ) * 255.0 );

	cv::Mat result;
	LUT( img, lut_matrix, result );

	return result;
}

/*void DAEDALUS::ApplyClahe(cv::Mat inputImage, cv::Mat& outputImage, int limit)
{
	Ptr<CLAHE> clahe = createCLAHE();
	clahe->setClipLimit(limit);
	clahe->apply(inputImage, outputImage);
}*/

void DAEDALUS::rotate(cv::Mat& src, double angle, cv::Point2f center, cv::Mat& dst)
{
    int len = std::max(src.cols, src.rows);
    // cv::Point2f pt(len/2., len/2.);
    cv::Mat r = cv::getRotationMatrix2D(center, angle, 1.0);

    cv::warpAffine(src, dst, r, cv::Size(len, len));
}

template < typename T >
std::string DAEDALUS::to_string(const T& n)
{
	std::ostringstream stm;
	stm << n;
	return stm.str();
}

// -----------------------------------------------------------------------------
std::vector<DAEDALUS::Blob> DAEDALUS::detectBlobs (cv::Mat &img, float bounds)
{
  std::vector<Blob> _blobs;
  
  for (int y = 0; y < img.rows; y++)
  {
    for (int x = 0; x < img.cols; x++)
    {
      if (img.at<uchar>(y, x) >= 200.0)  // is pixel valid?
      {
	bool blobFound = false;
	for (int i = 0; i < _blobs.size(); i++)
	{
	    if (abs(x - _blobs.at(i).pt[0]) <= bounds &&
	      abs(y - _blobs.at(i).pt[1]) <= bounds)
	    {
		_blobs.at(i).pt[0] = (
								(_blobs.at(i).pt[0] * _blobs.at(i).support) + x) / 
								(_blobs.at(i).support + 1);
		_blobs.at(i).pt[1] = (
								(_blobs.at(i).pt[1] * _blobs.at(i).support) + y) / 
								(_blobs.at(i).support + 1);
		_blobs.at(i).pt[2] = 1;
		_blobs.at(i).support++;
		blobFound = true;
		break;
	    }
	}
	if (!blobFound)
	{
	  Blob b;
	  b.pt[0] = x;
	  b.pt[1] = y;
	  b.pt[2] = 1;
	  b.support = 1;
	  _blobs.push_back(b);
	}
      }
    }
  }
  
  return _blobs;
}
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
std::vector<DAEDALUS::Blob> DAEDALUS::detectHLines (cv::Mat &img, float vertBound)
{
  std::vector<Blob> _hLines;
  
  for (int y = 0; y < img.rows; y++)
  {
    for (int x = 0; x < img.cols; x++)
    {
      if (img.at<uchar>(y, x) >= 200.0)  // is pixel valid?
      {
	bool hLineFound = false;
	for (int i = 0; i < _hLines.size(); i++)
	{
	    if ( abs(y - _hLines.at(i).pt[1]) <= vertBound && x > 250 )
	    {
		_hLines.at(i).pt[1] = (
								(_hLines.at(i).pt[1] * _hLines.at(i).support) + y) / 
								(_hLines.at(i).support + 1);
		_hLines.at(i).pt[2] = 1;
		_hLines.at(i).support++;
		hLineFound = true;
		break;
	    }
	}
	if (!hLineFound)
	{
	  Blob b;
	  b.pt[0] = img.cols / 2;
	  b.pt[1] = y;
	  b.pt[2] = 1;
	  b.support = 1;
	  _hLines.push_back(b);
	}
      }
    }
  }
  
  return _hLines;
}
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
std::vector<DAEDALUS::Blob> DAEDALUS::detectHorLine ( std::vector<Blob> _pts, int _vertDistance, int _support )
{
  std::vector<Blob> _horLines;
  std::vector<Blob> _ret;
  for ( int i = 0; i < _pts.size(); i++ )
  {
    bool inserted = false;
    for ( int j = 0; j < _horLines.size(); j++ )
    {
      if( _pts.at(i).pt[1] - _horLines.at(j).pt[1] < _vertDistance )
      {
		_horLines.at(j).pt[0] = ( 
								(_horLines.at(j).pt[0] * _horLines.at(j).support) + 
								_pts.at(i).pt[0] ) / 
								( _horLines.at(j).support + 1 );
		_horLines.at(j).pt[1] = (
								(_horLines.at(j).pt[1] * _horLines.at(j).support) +
								_pts.at(i).pt[1] ) /
								( _horLines.at(j).support + 1 );
		_horLines.at(j).support++;
		inserted = true;
		break;
      }
    }
    if ( !inserted )
    {
      Blob _hLine;
      _hLine.pt = _pts.at(i).pt;
      _hLine.support = 1;
      _horLines.push_back( _hLine );
    }
  }
  
  for ( int i = 0; i < _horLines.size(); i++ )
  {
    if ( _horLines.at(i).support > _support )
      _ret.push_back( _horLines.at(i) );
  }
  
  return _ret;
}
// -----------------------------------------------------------------------------

/*
// Function for blending multiple images
cv::Mat DAEDALUS::blendImages() {

	cv::Mat frame,frame32f;
	char filename[40];
	cv::Mat mean;
	const int count =1000;
	const int width  = 320;
	const int height = 240;
	cv::Mat resultframe = cv::Mat::zeros(height,width,CV_32FC3);
	for(int i = 0 ; i< count; i++){
		sprintf(filename,"/home/aadc/Desktop/images/base/%d.png",i);
		frame = imread(filename,CV_LOAD_IMAGE_COLOR);
		frame.convertTo(frame32f,CV_32FC3);
		resultframe +=frame32f;
    	cout << " i = " << i<<endl;
		//frame.release();
	}
resultframe *= (1.0/count);

imshow("",resultframe);
waitKey(0);
return 0;
  }
*/
