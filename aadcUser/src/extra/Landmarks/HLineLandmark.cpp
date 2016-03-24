#include "HLineLandmark.h"

using namespace cv;

// ----- Default Constructor ---------------------------------------------------
HLineLandmark::HLineLandmark (  ) :
	Landmark()
{
} 
// -----------------------------------------------------------------------------

// ----- Initializing Constructor ----------------------------------------------
HLineLandmark::HLineLandmark ( Vec3d u, Matx33d cov ) :
	Landmark( u, cov ),
	orientation(0)
{
} 
// -----------------------------------------------------------------------------

// ----- Destructor ------------------------------------------------------------
HLineLandmark::~HLineLandmark (  )
{

} 
// -----------------------------------------------------------------------------
