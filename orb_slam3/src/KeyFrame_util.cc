
/**
* Tariq updated - Dec, 2022
**/

#include "KeyFrame.h"

namespace ORB_SLAM3 {

void KeyFrame::AddMapObject(MapObject *pMO, int idx) {
    unique_lock<mutex> lock(mMutexObjects);
    mvpMapObjects[idx] = pMO;
}

void KeyFrame::EraseMapObjectMatch(const size_t &idx) {
    unique_lock<mutex> lock(mMutexObjects);
    mvpMapObjects[idx] = static_cast<MapObject *>(NULL);
}

void KeyFrame::EraseMapObjectMatch(MapObject *pMO) {
    int idx = pMO->GetIndexInKeyFrame(this);
    if (idx > 0)
        mvpMapObjects[idx] = static_cast<MapObject *>(NULL);
}

void KeyFrame::ReplaceMapObjectMatch(const size_t &idx, MapObject *pMO) {
    mvpMapObjects[idx] = pMO;
}

vector<ObjectDetection *> KeyFrame::GetObjectDetections() {
    unique_lock<mutex> lock(mMutexObjects);
    return mvpDetectedObjects;
}

vector<MapObject *> KeyFrame::GetMapObjectMatches() {
    unique_lock<mutex> lock(mMutexObjects);
    return mvpMapObjects;
}

}
