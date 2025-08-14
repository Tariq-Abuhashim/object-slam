/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

/**
* Tariq updated - Nov, 2022
**/

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Atlas.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "Initializer.h"
#include "Converter.h"
#include "ObjectDrawer.h"
#include "ObjectMapping.h" // Object processing thread

#include <mutex>

// COVINS
#include "comm/communicator.hpp"

namespace ORB_SLAM3
{

class System;
class Tracking;
class LoopClosing;
class Atlas;
class MapObject;
class ObjectMapping; // Object processing thread

class LocalMapping
{
public:
    LocalMapping(System* pSys, Atlas* pAtlas, ObjectDrawer* pObjectDrawer, const float bMonocular, bool bInertial, const string &_strSeqName=std::string());

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

	void SetObjectMapper(ObjectMapping* pObjectMapper);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);
    void EmptyQueue();

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

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    bool IsInitializing();
    double GetCurrKFTime();
    KeyFrame* GetCurrKF();

    std::mutex mMutexImuInit;

    Eigen::MatrixXd mcovInertial;
    Eigen::Matrix3d mRwg;
    Eigen::Vector3d mbg;
    Eigen::Vector3d mba;
    double mScale;
    double mInitTime;
    double mCostTime;
    bool mbNewInit;
    unsigned int mInitSect;
    unsigned int mIdxInit;
    unsigned int mnKFs;
    double mFirstTs;
    int mnMatchesInliers;

    bool mbNotBA1;
    bool mbNotBA2;
    bool mbBadImu;

    bool mbWriteStats;

    // not consider far points (clouds)
    bool mbFarPoints;
    float mThFarPoints;

#ifdef REGISTER_TIMES
    vector<double> vdKFInsert_ms;
    vector<double> vdMPCulling_ms;
    vector<double> vdMPCreation_ms;
    vector<double> vdLBA_ms;
    vector<double> vdKFCulling_ms;
    vector<double> vdLMTotal_ms;


    vector<double> vdLBASync_ms;
    vector<double> vdKFCullingSync_ms;
    vector<int> vnLBA_edges;
    vector<int> vnLBA_KFopt;
    vector<int> vnLBA_KFfixed;
    vector<int> vnLBA_MPs;
    int nLBA_exec;
    int nLBA_abort;
#endif

    #ifdef COVINS_MOD
    void SetComm(std::shared_ptr<Communicator> comm) {
        comm_ = comm;
    }

    auto IsCommInitialized()->bool {
        std::unique_lock<std::mutex> lock(mtx_comm_init_);
        return comm_init_;
    }
    auto SetCommInitialized()->void {
        std::unique_lock<std::mutex> lock(mtx_comm_init_);
        comm_init_ = true;
    }
    #endif

    // Object-SLAM
    KeyFrame* mpLastKeyFrame;
    std::list<MapObject*> mlpRecentAddedMapObjects;
    void GetNewObservations();
    void CreateNewMapObjects();
    void MapObjectCulling();
    void CreateNewObjectsFromDetections();
    void ProcessDetectedObjects();
    py::object pyOptimizer;
    py::object pyMeshExtractor;
    int nLastReconKFID;
    
    inline void SetDebug(const bool flag) {
    	std::cout << "[DEBUG]: LocalMapping debug flag has been set to TRUE\n";
		_debug = flag;
    }

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();
    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);
    cv::Matx33f ComputeF12_(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);
    cv::Matx33f SkewSymmetricMatrix_(const cv::Matx31f &v);

    System *mpSystem;

    bool mbMonocular;
    bool mbInertial;

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

    Atlas* mpAtlas;
    //Map* mpMap;
    ObjectDrawer* mpObjectDrawer; //DSP

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;
	ObjectMapping* mpObjectMapper; // Object processing thread

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    void InitializeIMU(float priorG = 1e2, float priorA = 1e6, bool bFirst = false);
    void ScaleRefinement();

    bool bInitializing;

    Eigen::MatrixXd infoInertial;
    int mNumLM;
    int mNumKFCulling;

    float mTinit;

    int countRefinement;

    //DEBUG
    ofstream f_lm;

    #ifdef COVINS_MOD
    std::shared_ptr<Communicator> comm_;
    std::set<KeyFrame*,KeyFrame::cmp_by_id> kf_out_buffer_;
    std::mutex mtx_comm_init_;
    bool comm_init_ = false;
    #endif
    
    // object-slam
    bool _debug;
    bool _use_python;
    bool _use_lidar;
    /*
    #ifdef NDEBUG
    bool _debug = false;
    #else
    bool _debug = true;
	#endif
	*/
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
