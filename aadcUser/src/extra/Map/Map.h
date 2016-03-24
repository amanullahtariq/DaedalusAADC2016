#ifndef _MAP_H
#define _MAP_H

#include <stdlib.h>
#include "Landmark.h"
#include "opencv/cv.hpp"
#include "opencv2/core.hpp"
#include "DAEDALUS_util.h"

#define PI 3.14159265359

using namespace DAEDALUS;
using namespace cv;

class Map
{
public:
	Map(Vec2d dimensions, Vec3d carPosition, int mToP, double achsisDistance, double associationDistance);
	~Map();

	void						correctionStep	(std::vector<Blob> sensorReading);
	void						updateStep		(double angle, double distance);
	std::vector<Landmark*>		getMap			();
	void						clear			();

private:
	std::vector<Landmark*>		_map;
	std::vector<Landmark*>		_map_temp;

	Vec2d						_dimensions;
	Vec3d						_carPos;
	int							_mToP;
	double						_achsisDistance;
	double						_assocDistance;
	

	Matx33d Q;
	Matx33d I;	

	Matx33d						computeMotionUpdate	( double pSteeringAngle, double pDistance );


};

#endif // _MAP_H