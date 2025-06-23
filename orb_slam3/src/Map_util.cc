
/**
* Tariq updated - Jun, 2022
* Tariq updated - Mar, 2023
**/

#include "Converter.h"
#include "Map.h"
#include <mutex>

namespace ORB_SLAM3
{

void Map::AddMapObject(MapObject *pMO)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapObjects.insert(pMO);
}

void Map::EraseMapObject(MapObject *pMO)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapObjects.erase(pMO);
}

vector<MapObject*> Map::GetAllMapObjects()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapObject*>(mspMapObjects.begin(), mspMapObjects.end());
}

MapObject* Map::GetMapObject(int object_id)
{
    unique_lock<mutex> lock(mMutexMap);
    for (auto mspMapObject : mspMapObjects)
    {
        if(mspMapObject->mnId != object_id)
            continue;
        return mspMapObject;
    }
    return NULL;
}

void Map::ApplyScaledRotationWithObjects(const cv::Mat &R, const float s, const bool bScaledVel, const cv::Mat t)
{
    unique_lock<mutex> lock(mMutexMap);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    cv::Mat Txw = cv::Mat::eye(4,4,CV_32F);
    R.copyTo(Txw.rowRange(0,3).colRange(0,3));

    cv::Mat Tyx = cv::Mat::eye(4,4,CV_32F);

    cv::Mat Tyw = Tyx*Txw;
    Tyw.rowRange(0,3).col(3) = Tyw.rowRange(0,3).col(3)+t;
    cv::Mat Ryw = Tyw.rowRange(0,3).colRange(0,3);
    cv::Mat tyw = Tyw.rowRange(0,3).col(3);
	
	// KeyFrames
    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(); sit!=mspKeyFrames.end(); sit++)
    {
        KeyFrame* pKF = *sit;
        cv::Mat Twc = pKF->GetPoseInverse();
        Twc.rowRange(0,3).col(3)*=s;
        cv::Mat Tyc = Tyw*Twc;
        cv::Mat Tcy = cv::Mat::eye(4,4,CV_32F);
        Tcy.rowRange(0,3).colRange(0,3) = Tyc.rowRange(0,3).colRange(0,3).t();
        Tcy.rowRange(0,3).col(3) = -Tcy.rowRange(0,3).colRange(0,3)*Tyc.rowRange(0,3).col(3);
        pKF->SetPose(Tcy);
        cv::Mat Vw = pKF->GetVelocity();
        if(!bScaledVel)
            pKF->SetVelocity(Ryw*Vw);
        else
            pKF->SetVelocity(Ryw*Vw*s);

    }
	// MapPoints
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(); sit!=mspMapPoints.end(); sit++)
    {
        MapPoint* pMP = *sit;
        pMP->SetWorldPos(s*Ryw*pMP->GetWorldPos()+tyw);
        pMP->UpdateNormalAndDepth();
    }
	// MapObjects
/*
    for(set<MapObject*>::iterator sit=mspMapObjects.begin(); sit!=mspMapObjects.end(); sit++)
    {
        MapObject* pMO = *sit;
		Eigen::Matrix4f SE3Two = pMO->GetPoseSE3();
		cv::Mat Two = Converter::toCvMat(SE3Two);
		Two.rowRange(0,3).col(3)*=s;
		cv::Mat Tyo = Tyw*Two;
		pMO->SetObjectPoseSE3(Converter::toMatrix4f(Tyo));
    }
*/

    mnMapChange++;
}

}

