/**
* Tariq updated - Oct, 2022
* Tariq updated - Jan, 2023
**/

#ifndef OBJECTDRAWER_H
#define OBJECTDRAWER_H

#include "Converter.h"
#include "Atlas.h"
#include "MapObject.h"
#include "MapDrawer.h"
#include "KeyFrame.h"
#include "ObjectRenderer.h"
#include <pangolin/pangolin.h>

/**
* Tariq updated - Jun, 2022
**/

namespace ORB_SLAM3
{

class KeyFrame;
class Map;
class MapDrawer;

class ObjectDrawer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ObjectDrawer(Atlas *pAtlas, MapDrawer *pMapDrawer, const string &strSettingPath);
    void SetRenderer(ObjectRenderer *pRenderer);
    void AddObject(MapObject *pMO);
    void ProcessNewObjects();
    void DrawObjects(bool bFollow, const Eigen::Matrix4f &Tec); // ORB_SLAM2
	void DrawObjects(bool bFollow); // ORB_SLAM3
    void DrawCuboid(MapObject *pMO);
    void SetCurrentCameraPose(const Eigen::Matrix4f &Tcw);
    std::list<MapObject*> mlNewMapObjects;
    Atlas *mpAtlas;
    MapDrawer *mpMapDrawer;
    ObjectRenderer *mpRenderer;
    float mViewpointF;
    std::mutex mMutexObjects;
    std::vector<std::tuple<float, float, float>> mvObjectColors;
    Eigen::Matrix4f SE3Tcw; // current camera pose
    Eigen::Matrix4f SE3TcwFollow; // pose of camera which our eye is attached
};

}


#endif //OBJECTDRAWER_H
