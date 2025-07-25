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
* Tariq updated - Oct, 2022
**/

#ifndef SYSTEM_H
#define SYSTEM_H

#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "ObjectDrawer.h"
#include "Atlas.h"
//#include "ObjectMapping.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "ImuTypes.h"
#include "Config.h"

// COVINS
#include "comm/communicator.hpp"

// Object-SLAM
#include "tcp_interface.hpp" // mission-systems
#include <pybind11/embed.h>
#include <pybind11/eigen.h>
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <numpy/arrayobject.h>
namespace py = pybind11;
class PyThreadStateLock
{
public:
    PyThreadStateLock()
    {
        state = PyGILState_Ensure();
    }

    ~PyThreadStateLock()
    {
        PyGILState_Release(state);
    }
private:
    PyGILState_STATE state;
};

namespace ORB_SLAM3
{

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev)
    {
        if(lev <= th)
            cout << str << endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};

class Viewer;
class FrameDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class ObjectMapping; // Object-SLAM

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4,
		IMU_RGBD=5
    };

    // File type
    enum eFileType{
        TEXT_FILE=0,
        BINARY_FILE=1,
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string(), const string &strLoadingFile = std::string());
    
    ~System() {
    	/* Object-SLAM */
    	finalize();  // Safe to call even if Python isn't initialized
    }

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, string filename="");

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");


    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear Atlas or the active map)
    void Reset();
    void ResetActiveMap();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    void SaveTrajectoryEuRoC(const string &filename);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);

	// Object-SLAM
	void SaveMapCurrentFrame(const string &dir, int frameId);
    void SaveEntireMap(const string &dir);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    // For debugging
    double GetTimeFromIMUInit();
    bool isLost();
    bool isFinished();

    void ChangeDataset();

#ifdef REGISTER_TIMES
    void InsertRectTime(double& time);

    void InsertTrackTime(double& time);
#endif

    /* Object-SLAM */
    inline void InitThread()
    {
        if (!PyEval_ThreadsInitialized())
        {
            PyEval_InitThreads();
        }
    };
	/* finalise the interpreter
	*/
    inline void finalize() {
    	if (Py_IsInitialized()) {
			//py::gil_scoped_acquire acquire; // acquire GIL
			// Clear Python objects before finalization
			pySequence = py::object();
			pyDecoder = py::object();
			pyCfg = py::object();
					
			// Finalize from main thread
			py::finalize_interpreter();
		
			cout << "[SLAM] Python interpreter finalized" << endl;
		}
	};
    py::object pyCfg;
    py::object pyDecoder;
    py::object pySequence;
    bool _use_python;

	/* Communications */
	boost::asio::ip::tcp::socket* socket_;
	//std::string server_;
	//std::string port_;

	//double roll=0, pitch=0, yaw=0; // drone orientation
	void setSocket(const std::string& server, const std::string& port);
	void call_python_function(const double& Time, const cv::Mat& image, 
								const std::vector<Point>& pointCloud, const Imu& imu);

	/* process the synchronized data
	*/
	void processSyncedData(const CloudData& lidar, const ImageData& image, dataQueue& imuQueue);
/*
	template <typename T1, typename T2>
	void processSyncedData(const SensorData<T1>& data1, const SensorData<T2>& data2) {
    	// get the pointcloud
		std::vector<Point> pointCloud = data1.getData().points;
		// get the image
		Image I = data2.getData();
		cv::Mat image(I.height, I.width, CV_8UC3, I.data.data()); // does not copy the data. If vecData goes out of scope or is modified, the cv::Mat will be affected.
    	//std::cout << "Processing data: Lidar time(" << data1.getTimestamp()/1e9 << "), Image time(" << data2.getTimestamp()/1e9 << ")" << std::endl;
		//std::cout << "Processing data: Lidar data(" << pointCloud.size() << "), Image data(" << I.height << "x" << I.width << ")" << std::endl;
		// call python function
		call_python_function(data2.getTimestamp(), image, pointCloud);
	};
*/

private:

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Atlas structure that stores the pointers to all KeyFrames and MapPoints.
    Atlas* mpAtlas;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;
	ObjectMapping* mpObjectMapper;  // Object-SLAM

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

#ifdef COVINS_MOD
    std::shared_ptr<Communicator> comm_;
    covins::TypeDefs::ThreadPtr thread_comm_;
#endif

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    ObjectDrawer* mpObjectDrawer;  // Object-SLAM

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
	std::thread* mptObjectMapping;  // Object-SLAM
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;
    bool mbResetActiveMap;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;
    
    // Object-slam
    void check_lapack_blas_linkage();
    void check_numpy();
    void check_open3d();
    void check_numba();
    void check_pytorch_cuda();
    void check_mmcv();
    void check_mmseg();
    void check_mmdet();
    void check_mmdet3d();
	void check_detectors();
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
