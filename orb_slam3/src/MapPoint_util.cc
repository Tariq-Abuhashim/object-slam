
/**
* updated - Jun, 2022
* updated - Aug, 2022
**/

#include "MapPoint.h"

namespace ORB_SLAM3
{

void MapPoint::SetOutlierFlag()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    mbOutlier = true;
}

bool MapPoint::isOutlier()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbOutlier;
}

}