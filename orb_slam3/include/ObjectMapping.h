
/**
* Tariq created - Apr, 2023
**/

#ifndef OBJECTMAPPING_H
#define OBJECTMAPPING_H

#include <mutex>
#include "Tracking.h"
#include "LocalMapping.h"
#include "ObjectDetection.h"

namespace ORB_SLAM3
{

class System;
class Tracking;
class LocalMapping;
class Atlas;
class MapObject;

class ObjectMapping 
{
public:
	ObjectMapping(System* pSys, Atlas *pAtlas, ObjectDrawer* pObjectDrawer, const float bMonocular, bool bInertial, const string &_strSeqName);

	void SetTracker(Tracking *pTracker);
	void SetLocalMapper(LocalMapping *pLocalMapper);

	void InsertKeyFrame(KeyFrame* pKF);
    void EmptyQueue();

	// Main function
    void Run();

	KeyFrame* mpLastKeyFrame;

	// Thread Synch
    void RequestStop();
    void RequestReset();
    void RequestResetActiveMap(Map* pMap);
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

protected:
	System* mpSystem;
	Atlas *mpAtlas;
	ObjectDrawer* mpObjectDrawer;
    Tracking* mpTracker;
	LocalMapping* mpLocalMapper;

	bool CheckNewKeyFrames();
    void GetNewObservations();
    void CreateNewMapObjects();
    void MapObjectCulling();
    void CreateNewObjectsFromDetections();
    void ProcessDetectedObjects();
    py::object pyOptimizer;
    py::object pyMeshExtractor;
    int nLastReconKFID;

	bool mbMonocular;
	bool mbInertial; 
	bool mbWITH_LIDAR;

	bool mbAcceptKeyFrames;
	std::mutex mMutexAccept;

	KeyFrame* mpCurrentKeyFrame;
	std::list<KeyFrame*> mlNewKeyFrames;
	std::list<MapObject*> mlpRecentAddedMapObjects;
	std::mutex mMutexNewKFs;

	void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

	bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

}; // ObjectMapping

} // ORB_SLAM3

#endif // OBJECTMAPPING_H