
/**
* Tariq created - Apr, 2023
**/

#include <iostream>
#include "ObjectMapping.h"
#include "Optimizer.h"

namespace ORB_SLAM3
{

ObjectMapping::ObjectMapping(System* pSys, Atlas *pAtlas, ObjectDrawer* pObjectDrawer, const float bMonocular, bool bInertial, const string &_strSeqName):
    mpSystem(pSys), mpAtlas(pAtlas), mpObjectDrawer(pObjectDrawer), mbMonocular(bMonocular), mbInertial(bInertial), 
	mbResetRequested(false), mbResetRequestedActiveMap(false), mbFinishRequested(false), mbFinished(true), 
    mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true), mbWITH_LIDAR(true)
{

	// Object-SLAM
	py::module optim  = py::module::import("reconstruct.optimizer");
    pyOptimizer = optim.attr("Optimizer")(pSys->pyDecoder, pSys->pyCfg);
    pyMeshExtractor = optim.attr("MeshExtractor")(pSys->pyDecoder, pSys->pyCfg.attr("optimizer").attr("code_len"), pSys->pyCfg.attr("voxels_dim"));
    //mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    nLastReconKFID = 0;

}

void ObjectMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void ObjectMapping::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void ObjectMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

void ObjectMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
}

bool ObjectMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

bool ObjectMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Object Mapping STOP" << endl;
        return true;
    }

    return false;
}

void ObjectMapping::ResetIfRequested()
{
    bool executed_reset = false;
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetRequested)
        {
            executed_reset = true;
			mlNewKeyFrames.clear();
			mlpRecentAddedMapObjects.clear();
			std::vector<MapObject*> pMOs = mpAtlas->GetAllMapObjects();
			for(MapObject* pMO : pMOs)
				mpAtlas->EraseMapObject(pMO);			
            cout << "OM: End reseting Object Mapping..." << endl;
        }

        if(mbResetRequestedActiveMap) {
            executed_reset = true;
			mlNewKeyFrames.clear();
            mlpRecentAddedMapObjects.clear();
			std::vector<MapObject*> pMOs = mpAtlas->GetAllMapObjects();
			for(MapObject* pMO : pMOs)
				mpAtlas->EraseMapObject(pMO);
            cout << "OM: End reseting Object Mapping..." << endl;
        }
    }
    if(executed_reset)
        cout << "OM: Reset free the mutex" << endl;

}

bool ObjectMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool ObjectMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

bool ObjectMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void ObjectMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}


/*
 * Main function
 */


void ObjectMapping::Run()
{

	mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames() && !mpLocalMapper->mbBadImu && !stopRequested())
        {

			{
        		unique_lock<mutex> lock(mMutexNewKFs);
        		mpCurrentKeyFrame = mlNewKeyFrames.front();
        		mlNewKeyFrames.pop_front();
    		}		

			if (mpTracker->mSensor == System::STEREO || mpTracker->mSensor == System::IMU_STEREO)
            {
                // Get new observations for map objects
                GetNewObservations();
                // Recent MapObjects Culling
                //MapObjectCulling();
                // Create new MapObjects
                CreateNewMapObjects();
            }
            else if (mpTracker->mSensor == System::MONOCULAR || mpTracker->mSensor == System::IMU_MONOCULAR)
            {
                //if (mpTracker->mState != Tracking::NOT_INITIALIZED)
				//if (mpAtlas->KeyFramesInMap()>50) // FIXME need to link to IMU initialisation too
                {
					// for monocular simulation with inertial and lidar (only create object after IMU is initialised)
					if (mbWITH_LIDAR)
					{
						//if (mpLastKeyFrame->GetMap()->GetIniertialBA2())
						if (mpAtlas->isImuInitialized())
						{
							// Get new observations for map objects
		            		GetNewObservations();
		            		// Recent MapObjects Culling
		            		//MapObjectCulling();
		            		// Create new MapObjects
		            		CreateNewMapObjects();
						}
					}
					else
					{	// for monocular only simulation
		                //if (mpAtlas->GetAllMapObjects().empty())
		                //    CreateNewObjectsFromDetections();
		                // reconstruction
		                //ProcessDetectedObjects();
					}
                }
            }

			int num_FixedKF_BA = 0;
            int num_OptKF_BA = 0;
            int num_MPs_BA = 0;
            int num_edges_BA = 0;
			bool mbAbortBA = false;

			//if (mpCurrentKeyFrame->GetMap()->isImuInitialized())
            //	Optimizer::LocalJointBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA,mpCurrentKeyFrame->GetMap());

			mpLastKeyFrame = mpCurrentKeyFrame;

        }
        else if(Stop() && !mpLocalMapper->mbBadImu)
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is idle
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}


/*
 * Tracking utils for stereo+lidar on KITTI
 */


void ObjectMapping::GetNewObservations()
{

	//cout << "LM: GetNewObservations." << endl;
	
    PyThreadStateLock PyThreadLock;

	//Verbose::PrintMess("LM: Estimating new poses for associated objects", Verbose::VERBOSITY_NORMAL);

    auto Tcw = Converter::toMatrix4f(mpCurrentKeyFrame->GetPose());
    auto mvpAssociatedObjects = mpCurrentKeyFrame->GetMapObjectMatches();
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();

	assert((int)mvpObjectDetections.size()==(int)mvpAssociatedObjects.size());

	Map* pCurrentMap = mpAtlas->GetCurrentMap();

    for (int i = 0; i < mvpObjectDetections.size(); i++)
    {
        auto det = mvpObjectDetections[i];
        if (det->isNew)
            continue;
        if (!det->isGood)
            continue;

        auto pMO = mvpAssociatedObjects[i];
        if (pMO)
        {
            // Tco obtained by transforming Two to camera frame
            Eigen::Matrix4f iniSE3Tco = Tcw * pMO->GetPoseSE3();
            g2o::SE3Quat Tco = Converter::toSE3Quat(iniSE3Tco);
            // Tco after running ICP, use Tco provided by detector
            Eigen::Matrix4f SE3Tco = pyOptimizer.attr("estimate_pose_cam_obj")
                    (det->SE3Tco, pMO->scale, det->SurfacePoints, pMO->GetShapeCode()).cast<Eigen::Matrix4f>();
            g2o::SE3Quat Zco = Converter::toSE3Quat(SE3Tco);
            // error
            Eigen::Vector3f dist3D = SE3Tco.topRightCorner<3, 1>() - iniSE3Tco.topRightCorner<3, 1>();
            Eigen::Vector2f dist2D; dist2D << dist3D[0], dist3D[2];
            Eigen::VectorXd e = (Tco.inverse() * Zco).log(); // was Eigen::Vector<double , 6>

            if (pMO->isDynamic()) // if associated with a dynamic object
            {
				cout << "LM: associated with a dynamic object." << endl;
                auto motion = pMO->SE3Tow * Tcw.inverse() * SE3Tco;
                float deltaT = (float)(mpCurrentKeyFrame->mnFrameId - mpLastKeyFrame->mnFrameId);
                auto speed = motion.topRightCorner<3, 1>() / deltaT;
                pMO->SetObjectPoseSE3(Tcw.inverse() * SE3Tco);
                pMO->SetVelocity(speed);
            }
            else // associated with a static object
            {
				cout << "LM: associated with a static object." << endl;
                if (dist2D.norm() < 1.0 && e.norm() < 1.5) // if the change of translation is very small, then it really is a static object
				//if (dist2D.norm() < 2.0 && e.norm() < 3.0) // Monocular
                {
                    det->SetPoseMeasurementSE3(SE3Tco);
                }
                else // if change is large, it could be dynamic object or false association
                {
                    // If just observed, assume it is dynamic
                    if (pMO->Observations() <= 2)
                    {
                        pMO->SetDynamicFlag();
                        auto motion = pMO->SE3Tow * Tcw.inverse() * SE3Tco;
                        float deltaT = (float)(mpCurrentKeyFrame->mnFrameId - mpLastKeyFrame->mnFrameId);
                        auto speed = motion.topRightCorner<3, 1>() / deltaT;
                        pMO->SetObjectPoseSE3(Tcw.inverse() * SE3Tco);
                        pMO->SetVelocity(speed);	
                        pCurrentMap->mnDynamicObj++;
                    }
                    else
                    {
                        det->isNew = true;
                        mpCurrentKeyFrame->EraseMapObjectMatch(i);
                        pMO->EraseObservation(mpCurrentKeyFrame);
                    }
                }
            }
        }
    }
}

void ObjectMapping::MapObjectCulling()
{

	//Verbose::PrintMess("LM: Culling map objects", Verbose::VERBOSITY_NORMAL);

    // Check Recent Added MapObjects
    list<MapObject*>::iterator lit = mlpRecentAddedMapObjects.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    const int cnThObs = 2;

	Map* pCurrentMap = mpAtlas->GetCurrentMap();

    // Treat static and dynamic objects differently
    while(lit != mlpRecentAddedMapObjects.end())
    {
        MapObject* pMO = *lit;
        if (pMO->isDynamic())
        {
            if ((int) nCurrentKFid - (int) pMO->mpNewestKF->mnId  >= 2)
            {
                pMO->SetBadFlag();
                lit = mlpRecentAddedMapObjects.erase(lit); // FIXME Tariq: this will result in two objects getting removed
                pCurrentMap->mnDynamicObj--;
            }
        }

        if(pMO->isBad())
        {
            lit = mlpRecentAddedMapObjects.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMO->mnFirstKFid) >= 2 && pMO->Observations() <= cnThObs)
        {
            pMO->SetBadFlag();
            lit = mlpRecentAddedMapObjects.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMO->mnFirstKFid) >= 3)
            lit = mlpRecentAddedMapObjects.erase(lit);
        else
            lit++;
    }

    // Dynamic objects that aren't recently added
    if (pCurrentMap->mnDynamicObj > 0)
    {
        std::vector<MapObject*> pMOs = mpAtlas->GetAllMapObjects();
        for (MapObject *pMO : pMOs)
        {
            if (pMO->isDynamic())
            {
                if ((int) nCurrentKFid - (int) pMO->mpNewestKF->mnId  >= 2)
                {
                    pMO->SetBadFlag();
                    pCurrentMap->mnDynamicObj--;
                }
            }
        }
    }
}

void ObjectMapping::CreateNewMapObjects()
{

	cout << "LM: CreateNewMapObjects." << endl;

    PyThreadStateLock PyThreadLock;

	//Verbose::PrintMess("LM: Creating new map objects", Verbose::VERBOSITY_DEBUG);

    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();
	if (mvpObjectDetections.size() < 1)
	{
		cout << "    returned because no object detections." << endl;
		return;
	}
	
	auto SE3Twc = Converter::toMatrix4f(mpCurrentKeyFrame->GetPoseInverse());

	Map* pCurrentMap = mpAtlas->GetCurrentMap();

	int count = 0;

    for (int i = 0; i < mvpObjectDetections.size(); i++)
    {
        // This might happen when a new KF is created in Tracking thread
/*
        if (mbAbortBA)
		{
			cout << "      returned because mbAbortBA." << endl;
            return;
		}
*/

        auto det = mvpObjectDetections[i];

        if (det->nRays == 0)
        {
			cout << "      continued detection has no rays" << endl;
            continue;
		}
        if (!det->isNew)
        {
			cout << "      continued detection is not new" << endl;
            continue;
		}
        auto pyMapObject = pyOptimizer.attr("reconstruct_object")
                (det->Sim3Tco, det->SurfacePoints, det->RayDirections, det->DepthObs);
        if (!pyMapObject.attr("is_good").cast<bool>())
		{
			cout << "      continued object is not good" << endl;
            continue;
		}
/*
        if (mbAbortBA)
        {
			cout << "      returned because mbAbortBA." << endl;
            return;
		}
*/
        auto Sim3Tco = pyMapObject.attr("t_cam_obj").cast<Eigen::Matrix4f>();
        det->SetPoseMeasurementSim3(Sim3Tco);
        // Sim3, SE3, Sim3
        Eigen::Matrix4f Sim3Two = SE3Twc * Sim3Tco;
        auto code = pyMapObject.attr("code").cast<Eigen::VectorXf>(); // was Eigen::Vector<float, 64>
        auto pNewObj = new MapObject(Sim3Two, code, det->Box, mpCurrentKeyFrame, pCurrentMap);

        auto pyMesh = pyMeshExtractor.attr("extract_mesh_from_code")(code);
        pNewObj->vertices = pyMesh.attr("vertices").cast<Eigen::MatrixXf>();
        pNewObj->faces = pyMesh.attr("faces").cast<Eigen::MatrixXi>();

        pNewObj->AddObservation(mpCurrentKeyFrame, i);
        mpCurrentKeyFrame->AddMapObject(pNewObj, i);
        pCurrentMap->AddMapObject(pNewObj);
        mpObjectDrawer->AddObject(pNewObj);
        mlpRecentAddedMapObjects.push_back(pNewObj);

		count++;
		
    }
	//Verbose::PrintMess("LM: Finished new objects creation.", Verbose::VERBOSITY_NORMAL);
	cout << "      reconstructed " << count << " objects from " << mvpObjectDetections.size() << " detections." << endl;
}

} // ORB_SLAM3