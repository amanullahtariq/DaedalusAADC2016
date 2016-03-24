#ifndef _DAEDALUS_UTIL_H_
#define _DAEDALUS_UTIL_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"


namespace DAEDALUS
{
	struct Blob
	{
		cv::Vec3d	pt;
		int			support;
	};

	cv::Mat correctGamma( cv::Mat& img, double gamma );
	// void ApplyClahe(cv::Mat inputImage, cv::Mat& outputImage, int limit);
	void rotate(cv::Mat& src, double angle, cv::Point2f center, cv::Mat& dst);
	std::vector<Blob> detectBlobs ( cv::Mat &img, float bounds );
	std::vector<Blob> detectHorLine ( std::vector<Blob> _pts, int _vertDistance, int _support );
	std::vector<Blob> detectHLines (cv::Mat &img, float vertBound);
	template < typename T >
	std::string to_string(const T& n);
	enum Maneuver
	{
	  straight,
	  left,
	  right,
	  parallel_parking,
	  cross_parking,
	  pull_out_left,
	  pull_out_right,
	  change_lane_left,
	  change_lane_right,
	  stop
	};
	//cv::Mat		blendImages();
}

#endif // _DAEDALUS_UTIL_H_
