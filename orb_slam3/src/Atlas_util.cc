
/**
* Tariq  - Jun, 2022
**/

#include "Map.h"
#include <mutex>

#include "Atlas.h"

namespace ORB_SLAM3
{

void Atlas::AddMapObject(MapObject *pMO)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->mspMapObjects.insert(pMO);
}

void Atlas::EraseMapObject(MapObject *pMO)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->mspMapObjects.erase(pMO);
}

vector<MapObject*> Atlas::GetAllMapObjects()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return vector<MapObject*>(mpCurrentMap->mspMapObjects.begin(), mpCurrentMap->mspMapObjects.end());
}

MapObject* Atlas::GetMapObject(int object_id)
{
    unique_lock<mutex> lock(mMutexAtlas);
    for (auto mspMapObject : mpCurrentMap->mspMapObjects)
    {
        if(mspMapObject->mnId != object_id)
            continue;
        return mspMapObject;
    }
    return NULL;
}




}