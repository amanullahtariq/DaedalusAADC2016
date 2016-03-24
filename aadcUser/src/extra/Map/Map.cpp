#include "Map.h"

// -----------------------------------------------------------------------------
Map::Map(	Vec2d dimensions, Vec3d carPosition, int mToP, double achsisDistance,
			double associationDistance) :
_dimensions(dimensions),
_carPos(carPosition),
_mToP(mToP),
_achsisDistance(achsisDistance),
_assocDistance(associationDistance)
{
	Q = Matx33d(	0.1, 0.2, 0,
    				0.2, 0.1, 0,
    				0, 0, 1 );
    I = Matx33d(	1, 0, 0,
    				0, 1, 0,
    				0, 0, 1 );
}
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
Map::~Map()
{
	_map.clear();
	_map_temp.clear();
}
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
void Map::correctionStep (std::vector<Blob> sensorReading)
{

	_map_temp.clear();
	for ( int i = 0; i < sensorReading.size(); i++ )
	{
		// Landmark association
		double distance = DBL_MAX;
		int index = -1;
		for ( int j = 0; j < _map.size(); j++ )
		{
			double d = norm( sensorReading.at(i).pt, _map.at(j)->mean );
			if ( d < distance )
			{
				index = j;
				distance = d;
			}
		}
		
		// Kalman Update (Correction step)
		if( distance <= _assocDistance && index >= 0 )
		{
			// TODO uncomment the following lines for full Kalman step
			//Matx33d temp = _map.at(index)->covariance + Q;
			//Matx33d K = _map.at(index)->covariance * temp.inv();
			//_map.at(index)->mean = _map.at(index)->mean +
			//					(K * 
			//					(sensorReading.at(i).pt - _map.at(index)->mean)
			//					);
			//_map.at(index)->covariance = ( I - K ) *
			//							_map.at(index)->covariance;
			
			_map.at(index)->mean = _map.at(index)->mean +
								(0.5 * 
								(sensorReading.at(i).pt - _map.at(index)->mean)
								);

			_map.at(index)->fitness++;
			_map_temp.push_back(_map.at(index));
			_map.erase(_map.begin() + index);
		}
		// if no association exists, create a new landmark
		else
		{
			Landmark* _landmark = new Landmark( sensorReading.at(i).pt, Q );
			_landmark->age = 0;
			_landmark->fitness = 1;
			_map_temp.push_back(_landmark);
		}
	}
	
    _map.insert( _map.end(), _map_temp.begin(), _map_temp.end() );

}
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
void Map::updateStep (double angle, double distance)
{
	Matx33d R = computeMotionUpdate( angle, distance );
    _map_temp.clear();
    for ( int i = 0; i < _map.size(); i++ )
    {
      // Kalman Update (Prediction step)
      _map.at(i)->mean = R * _map.at(i)->mean;
      // _map.at(i)->covariance = (R * ( _map.at(i)->covariance * R.t() ) ) + 
      // 							Q;
      _map.at(i)->age++;
      if ( norm( _map.at(i)->mean, _carPos ) > 1200 || (_map.at(i)->age > 60 && _map.at(i)->fitness < 20))
      {
      	continue;
      }

      _map_temp.push_back(_map.at(i));
    }
    _map.clear();
    _map.insert( _map.begin(), _map_temp.begin(), _map_temp.end() );
    //_map = _map_temp;

}
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
std::vector<Landmark*> Map::getMap ()
{
  return _map;
}
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
void Map::clear ()
{
  _map.clear();
  _map_temp.clear();
}
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
/*
 * Motion model for an Ackerman drive
 * pSteeringAngle :	Steering angle of the front wheels
 *					(90deg is forward direction)
 * pDistance :		Driven distance with pSteeringAngle
 * dx :				change of x position
 * dy :				change of y position
 * alpha :			change of heading
 * R :				turning radius
 */
Matx33d Map::computeMotionUpdate( double pSteeringAngle, double pDistance )
{
	double r;
	double icr_x, icr_y;
	double alpha;
	Matx33d R, T1, T2;


	
	
	if (abs(pSteeringAngle - 90) < 1)
	{
		alpha	= 0;
		r		= 0;

		R = Matx33d(	1,	0,	0,
						0,	1,	pDistance * _mToP,
						0,	0,	1);
	}
	else
	{		
		double _phi		= abs( pSteeringAngle - 90. ) * PI / 180.;	// in rad
		r				= _achsisDistance / tan(_phi);				// in m
		r				= pSteeringAngle > 90. ? -r : r;
		alpha			= pDistance / r; 						// in radians

		R = Matx33d(	cos(alpha),	-sin(alpha),	0,
						sin(alpha),	cos(alpha),		0,
						0,			0,				1);

		// printf("Main: r: %f a: %f\n", r, alpha);
	}
	
	icr_x = _carPos[0] - ( r * _mToP );
	icr_y = _carPos[1];
	

	// printf("x: %f, y: %f, r:%f\n", icr_x, icr_y, r*_mToP);
	

	T1 = Matx33d(	1, 0, -icr_x,
					0, 1, -icr_y,
					0, 0, 1);

	T2 = Matx33d(	1, 0, icr_x,
					0, 1, icr_y,
					0, 0, 1);

	return (T2 * ( R * T1 ));
}
// -----------------------------------------------------------------------------