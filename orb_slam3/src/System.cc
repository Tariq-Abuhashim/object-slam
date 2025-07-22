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
* Tariq updated - Jun, 2022
* Tariq updated - Jul, 2022
* Tariq updated - Nov, 2022
* Tariq updated - Jul, 2023
**/

#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include <libgen.h>
#include <pybind11/pybind11.h>

// COVINS
//#include <comm/communicator.hpp> // for NO_LOOP_FINDER

bool has_suffix(const std::string &str, const std::string &suffix) {
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace ORB_SLAM3
{

Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, const int initFr, const string &strSequence, const string &strLoadingFile):
    mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false), mbResetActiveMap(false),
    mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "[SLAM] Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;
    else if(mSensor==IMU_MONOCULAR)
        cout << "Monocular-Inertial" << endl;
    else if(mSensor==IMU_STEREO)
        cout << "Stereo-Inertial" << endl;

	bool loadedAtlas = false;
	
	
	/* Check settings file */

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened()) {
       cerr << "[SLAM] Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    
	/* Object-slam python stuff */
	
	bool use_python = true;
	if(fsSettings["UsePython"].isInt()) {
		 use_python = (int)fsSettings["UsePython"] != 0;
	}
	
	if(use_python) {

			setenv("OMP_NUM_THREADS", "1", 1);
			setenv("OPENBLAS_NUM_THREADS", "1", 1);
			setenv("MKL_NUM_THREADS", "1", 1);
			setenv("NUMBA_NUM_THREADS", "1", 1);
			setenv("TOKENIZERS_PARALLELISM", "false", 1);
			setenv("NUMBA_DISABLE_JIT", "1", 1);  // Disable JIT compilation
			setenv("NUMBA_THREADING_LAYER", "safe", 1);  // Use safe threading (safe, workqueue)
			//setenv("PYTHONNOUSERSITE", "1", 1);  // Ignore user-site packages
			setenv("OPEN3D_CPU_PARALLEL_POLICY", "0", 1);  // Disable TBB parallelization in Open3D
			setenv("PYTORCH_NO_CUDA_MEMORY_CACHING", "1", 1);
			setenv("CUDA_LAUNCH_BLOCKING", "1", 1);  // forces CUDA errors to appear synchronously
			setenv("TORCH_USE_RTLD_GLOBAL", "1", 1);  // Helps with CUDA symbol loading

			setenv("CUDA_MODULE_LOADING", "LAZY", 1);

            setenv("PYTHONPATH", "/home/mrt/dev/object-slam/orb_slam3/Thirdparty/mmdetection3d", 1);
			//setenv("PYTHONPATH", "/home/mrt/.local/lib/python3.8/site-packages", 1);

		try {
			/* Python environment verification */
			if (Py_IsInitialized()) {
				cerr << "[Pybind] Warning: Python interpreter already initialized!" << endl;
			}
			
			std::cout << "[Pybind] Starting Python interpreter ..." << std::endl;
			py::initialize_interpreter();
			
			std::cout << "[Pybind] Initialise python thread ..." << std::endl;
            InitThread();  // This should be called right after interpreter initialization
			
			{
				py::gil_scoped_acquire acquire;
				
				/* System checks */
				
				std::cout << "[Pybind] Import sys module ..." << std::endl;
				py::module sys = py::module::import("sys");


				std::string version = py::str(sys.attr("version"));
				std::string executable = py::str(sys.attr("executable"));
				py::print("sys.path =", sys.attr("path"));
				std::cout << "[Pybind] Python version: " << version << std::endl;
				std::cout << "[Pybind] Python executable: " << executable << std::endl;
				
				/* Add executable directory to path, avoid hard coded paths
				   determines the directory where the current executable is located and 
				   adds it to Python's module search path (sys.path) */
				//char buf[PATH_MAX];
				//ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf)-1);
				//if (len != -1) {
				//	buf[len] = '\0'; // Null-Terminates the String
				//	string exe_dir = dirname(buf); // Extracts Directory Path
				//	sys.attr("path").attr("append")(exe_dir); // Adds to Python Path
				//}
				
				std::cout << "[Pybind] Verify critical package versions ..." << std::endl;
				
				/* Lapack and Blas */
				check_lapack_blas_linkage();
				
				/* numpy */
				check_numpy();
				
				/* open3d */
				check_open3d();
				
				/* numba */
				check_numba();
        		
				/* pytorch and CUDA */	    
				check_pytorch_cuda();
				
				/* mmcv */
				check_mmcv();

				/* mmseg */
				check_mmseg();
				
				/* mmdet */
				check_mmdet();
				
				/* mmdet3d */
				check_mmdet3d();

				/* custom detectors 2D/3D */
				check_detectors();

				/* Add current directory */
				
				std::cout << "[Pybind] Add current directory ..." << std::endl;
				sys.attr("path").attr("append")("./");

				/* Import object-slam modules */
				
				std::cout << "[Pybind] Import utils module ..." << std::endl;
				py::module io_utils = py::module::import("reconstruct.utils");

				std::cout << "[Pybind] Getting python configs ..." << std::endl;
				if(!fsSettings["DetectorConfigPath"].empty()) { 
					std::string pyCfgPath = fsSettings["DetectorConfigPath"].string();
					pyCfg = io_utils.attr("get_configs")(pyCfgPath);
					cout << "[Pybind] Loaded Python config from: " << pyCfgPath << endl;
				} else {
					throw runtime_error("DetectorConfigPath not specified in settings");
				}
					
				std::cout << "[Pybind] Getting deepsdf decoder ..." << std::endl;    
				pyDecoder = io_utils.attr("get_decoder")(pyCfg);

				try {
					py::gil_scoped_acquire acquire;  // Hold GIL for the entire block

					// Enable fault handler for better crash logs
					py::module::import("faulthandler").attr("enable")();

					// Ensure Python path includes mmdetection3d
					py::module::import("sys").attr("path").attr("append")("/home/mrt/dev/object-slam/orb_slam3/Thirdparty/mmdetection3d");

					// Initialize PyTorch and CUDA
					py::module torch = py::module::import("torch");
					if (torch.attr("cuda").attr("is_available")().cast<bool>()) {
						torch.attr("zeros")(1).attr("cuda")();  // Force CUDA init
					} else {
						std::cerr << "[WARNING] CUDA not available. Running on CPU." << std::endl;
					}

					// Verify Shapely
					try {
						py::module::import("shapely").attr("__version__");
					} catch (...) {
						std::cerr << "Shapely initialization failed" << std::endl;
						exit(1);
					}

					// Initialize mmcv and mmdet3d
					py::module::import("mmcv");
					py::module::import("mmdet3d");
					py::module::import("mmdet3d.models");

					// Call the Python function
					pySequence = py::module::import("reconstruct").attr("get_sequence")(strSequence, pyCfg);
					if (pySequence.is_none()) {
						std::cerr << "[SLAM] Error: get_sequence() returned None" << std::endl;
						exit(-1);
					}
					std::cout << "[Pybind] Sequence class: " << py::str(pySequence.get_type()).cast<std::string>() << std::endl;
				} catch (const py::error_already_set &e) {
					std::cerr << "[Pybind] Python error:\n" << e.what() << std::endl;
					// Re-throw or handle gracefully
				} catch (const std::exception &e) {
					std::cerr << "[C++] Exception: " << e.what() << std::endl;
				}

				/* Initialise */
				//std::cout << "[Pybind] Initialise python thread ..." << std::endl;
				//InitThread(); // Ensure proper GIL management in this function.

				/* Release GIL safely */
				//py::gil_scoped_release release; // Only release GIL if needed and safe here.
			}
			cout << "[Pybind] Python integration successful" << endl;
		} catch (const py::error_already_set& e) {
			std::cerr << "[Pybind] Python error: " << e.what() << std::endl;
			if(Py_IsInitialized()) {
				PyErr_Print();
				py::finalize_interpreter();  // Clean up if initialization failed
			}
			exit(-1);
		} catch (const std::exception& e) {
			std::cerr << "[Pybind] Standard error: " << e.what() << std::endl;
			if(Py_IsInitialized()) {
				py::finalize_interpreter();
			}
			exit(-1);
		}
	} else {
		std::cout << "[Pybind] Python integration disabled" << std::endl;
	}
	
	/* Load ORB Vocabulary */
    cout << endl << "[SLAM] Loading ORB Vocabulary. This could take a while..." << endl;
    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "[SLAM] Wrong path to vocabulary. " << endl;
        cerr << "[SLAM] Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "[SLAM] Vocabulary loaded!" << endl << endl;

    /* Create KeyFrame Database */
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
    
     /* Create the Atlas */
    mpAtlas = new Atlas(0);
    if (mSensor==IMU_STEREO || mSensor==IMU_MONOCULAR || mSensor==IMU_RGBD)
        mpAtlas->SetInertialSensor();
        
    /* Create Drawers. These are used by the Viewer */
	cout << "[SLAM] Setting drawers ..." << endl;
	cout << strSettingsFile << endl;
    mpFrameDrawer = new FrameDrawer(mpAtlas);
    mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile);
    mpObjectDrawer = new ObjectDrawer(mpAtlas, mpMapDrawer, strSettingsFile); // FIXME freezes viewer with bStepByStep
    mpMapDrawer->SetObjectDrawer(mpObjectDrawer);
    
    /* Initialize the Tracking thread */
    //(it will live in the main thread of execution, the one that called this constructor)
    cout << "[SLAM] Seq. Name: " << strSequence << endl;
	cout << "[SLAM] Setting tracker ..." << endl;
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, strSequence);

	/* Initialize the Local Mapping thread and launch */
	cout << "[SLAM] Setting local mapper ..." << endl;
	cout << strSettingsFile << endl;
    mpLocalMapper = new LocalMapping(this, mpAtlas, mpObjectDrawer, mSensor==MONOCULAR || mSensor==IMU_MONOCULAR, mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO, strSequence);
	mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run,mpLocalMapper);
    mpLocalMapper->mThFarPoints = fsSettings["ThDepth"];
    if(mpLocalMapper->mThFarPoints!=0) {
        cout << "[SLAM] Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
        mpLocalMapper->mbFarPoints = true;
    }
    else {
        mpLocalMapper->mbFarPoints = false;
	}
        
    /* Initialize the object Mapping thread and launch */
	//cout << "[SLAM] Setting objects mapper ..." << endl;
    //mpObjectMapper = new ObjectMapping(this, mpAtlas, mpObjectDrawer, mSensor==MONOCULAR || mSensor==IMU_MONOCULAR, mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO, strSequence);
    //mptObjectMapping = new thread(&ORB_SLAM3::ObjectMapping::Run, mpObjectMapper);
    
    /* Initialize the Loop Closing thread and launch  */
	// FIXME currently, loop closing is only for stereo (kitti) - Object-SLAM
	cv::FileNode node = fsSettings["loopClosing"];
    bool activeLC = true;
    if(!node.empty()) {
        activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
    }
	if (mSensor == STEREO || mSensor==IMU_STEREO) {
		cout << "[SLAM] Setting loop closer ..." << endl;
    	mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR, activeLC); // mSensor!=MONOCULAR);
    	mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);
	}
	else {
		mpLoopCloser = nullptr;
	}
	
	/* Set pointers between threads */

	cout << "[SLAM] Set pointers between threads in mpTracker ..." << endl;
    mpTracker->SetLocalMapper(mpLocalMapper);
    #ifdef COVINS_MOD
    #ifndef NO_LOOP_FINDER
    mpTracker->SetLoopClosing(mpLoopCloser);
    #endif
    #else
    mpTracker->SetLoopClosing(mpLoopCloser);
    #endif

	cout << "[SLAM] Set pointers between threads in mpLocalMapper ..." << endl;
    mpLocalMapper->SetTracker(mpTracker);
//	mpLocalMapper->SetObjectMapper(mpObjectMapper); // Object-SLAM : new object processing thread
    #ifdef COVINS_MOD
    #ifndef NO_LOOP_FINDER
    mpLocalMapper->SetLoopCloser(mpLoopCloser);
    #endif
    #else
    mpLocalMapper->SetLoopCloser(mpLoopCloser);
    #endif 

	cout << "[SLAM] Set pointers between threads in mpLoopCloser ..." << endl;
	if (mpLoopCloser) {
		mpLoopCloser->SetTracker(mpTracker);
    #ifdef COVINS_MOD
    #ifndef NO_LOOP_FINDER
    	mpLoopCloser->SetLocalMapper(mpLocalMapper);
    #endif
    #else
    	mpLoopCloser->SetLocalMapper(mpLocalMapper);
    #endif
	}
	
	// Object-SLAM
	//cout << "[SLAM] Set pointers between threads in mpObjectMapper ..." << endl;
	//mpObjectMapper->SetTracker(mpTracker);
    //mpObjectMapper->SetLocalMapper(mpLocalMapper);	
	
	/* Initialize the Viewer thread and launch */
	
	cout << "starting the viewer ..." << endl;
    if(bUseViewer) {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpObjectDrawer, mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
		if (mpLoopCloser)
        	mpLoopCloser->mpViewer = mpViewer;
        mpViewer->both = mpFrameDrawer->both;
	}
	
	/* Set communicator and the backend */
	
    #ifdef COVINS_MOD
    std::cout << ">>> COVINS: Initialize communicator" << std::endl;
    comm_.reset(new Communicator(covins_params::sys::server_ip,covins_params::sys::port,mpAtlas));
    std::cout << ">>> COVINS: Start comm thread" << std::endl;
    thread_comm_.reset(new std::thread(&Communicator::Run,comm_));
    // Get ID from back-end
    std::cout << ">>> COVINS: wait for back-end response" << std::endl;
    while(comm_->GetClientId() < 0){
        usleep(1000); //wait until ID is received from server
    }
    std::cout << ">>> COVINS: client id: " << comm_->GetClientId() << std::endl;
    mpLocalMapper->SetComm(comm_); // Pass to mapping
    #endif
    
     /* Set verbosity */
	//Verbose::eLevel th = fsSettings["verbose"];
    Verbose::SetTh(Verbose::VERBOSITY_DEBUG);  // VERBOSITY_QUIET, VERBOSITY_NORMAL, VERBOSITY_DEBUG   
	
	// Object-SLAM
	//PyEval_ReleaseThread(PyThreadState_Get());

	cout << "Object-slam is alive ..." << endl;

}

void System::check_lapack_blas_linkage() {
    try {
        py::module numpy_config = py::module::import("numpy.__config__");

        auto blas_info = numpy_config.attr("get_info")("blas_opt_info");
        auto lapack_info = numpy_config.attr("get_info")("lapack_opt_info");

        std::cout << "[Pybind] NumPy BLAS linkage: ";
        if (py::len(blas_info)) {
            std::cout << py::str(blas_info).cast<std::string>() << std::endl;
        } else {
            std::cout << "No optimized BLAS linked" << std::endl;
        }

        std::cout << "[Pybind] NumPy LAPACK linkage: ";
        if (py::len(lapack_info)) {
            std::cout << py::str(lapack_info).cast<std::string>() << std::endl;
        } else {
            std::cout << "No optimized LAPACK linked" << std::endl;
        }

    } catch (const py::error_already_set &e) {
        std::cerr << "[Pybind] Python exception while checking LAPACK/BLAS linkage:\n"
                  << e.what() << std::endl;
    }
}

void System::check_numpy() {
	py::module numpy = py::module::import("numpy");
	string numpy_ver = numpy.attr("__version__").cast<string>();
	std::cout << "[Pybind] NumPy version: " << numpy_ver << std::endl;        
	//if (numpy_ver < "1.21.3") { // 1.21.3
	//	throw runtime_error("[Pybind] NumPy version too old - requires >= 1.19.5");
	//}
}

void System::check_open3d() {
	py::module open3d = py::module::import("open3d");
	string open3d_ver = open3d.attr("__version__").cast<string>();
	std::cout << "[Pybind] Open3d version: " << open3d_ver << std::endl;        
	//if (open3d_ver != "0.18.0") { // 0.18.0
	//	throw runtime_error("[Pybind] Requires PyTorch version == 0.18.0");
	//}
}

void System::check_numba() {
	py::module numba = py::module::import("numba");
	string numba_ver = numba.attr("__version__").cast<string>();
	if (numba_ver != "0.53.0") { // 0.53.0
		throw runtime_error("[SLAM] Numba version required == 0.53.0");
	}
	std::cout << "[Pybind] Numba version: " << numba_ver << std::endl; 
	numba.attr("config").attr("THREADING_LAYER") = "workqueue";
	numba.attr("config").attr("DISABLE_JIT") = true;
}

void System::check_pytorch_cuda() {
	py::module torch = py::module::import("torch");
	std::string torch_ver = torch.attr("__version__").cast<std::string>();
	std::cout << "[Pybind] PyTorch version: " << torch_ver << std::endl;
	bool torch_cuda = torch.attr("cuda").attr("is_available")().cast<bool>();
	std::cout << "[Pybind] CUDA available: " << torch_cuda << std::endl;
	if (torch_ver.rfind("1.12.0", 0) != 0) {  // 1.12.0a0+git67ece03
		throw std::runtime_error("[SLAM] Requires PyTorch version == 1.12.0");
	}
}

void System::check_mmcv() {
	py::module mmcv = py::module::import("mmcv");
	std::string mmcv_ver = mmcv.attr("__version__").cast<std::string>();
	std::cout << "[Pybind] MMCV version: " << mmcv_ver << std::endl;
	if (mmcv_ver != "1.7.0") {  // 1.7.0
		throw std::runtime_error("[Pybind] Requires MMCV version == 1.0.0rc6");					
	}
}
				
void System::check_mmseg() {
	py::module mmseg = py::module::import("mmseg");
	std::string mmseg_ver = mmseg.attr("__version__").cast<std::string>();
	std::cout << "[Pybind] MMSegmentation version: " << mmseg_ver << std::endl;
	if (mmseg_ver != "0.30.0") {  // 0.30.0
		throw std::runtime_error("[Pybind] Requires MMSegmentation version == 0.30.0");
	}
}				

void System::check_mmdet() {
	py::module mmdet = py::module::import("mmdet");
	std::string mmdet_ver = mmdet.attr("__version__").cast<std::string>();
	std::cout << "[Pybind] MMDetection version: " << mmdet_ver << std::endl;
	if (mmdet_ver != "2.28.2") {  // 2.28.2
		throw std::runtime_error("[Pybind] Requires MMDetection version == 2.28.2");
	}
}
				
void System::check_mmdet3d() {
	py::module mmdet3d = py::module::import("mmdet3d");
	std::string mmdet3d_ver = mmdet3d.attr("__version__").cast<std::string>();
	std::cout << "[Pybind] MMDetection3D version: " << mmdet3d_ver << std::endl;
	if (mmdet3d_ver != "1.0.0rc6") {  // 1.0.0rc6
		throw std::runtime_error("[Pybind] Requires PyTorch version == 1.0.0rc6");
	}
}

void System::check_detectors() {
    std::cout << "[Pybind] Checking detectors..." << std::endl;

    if (!Py_IsInitialized()) {
        std::cerr << "[Pybind] Python not initialized!" << std::endl;
        throw std::runtime_error("Python not initialized");
    }

    try {
        py::gil_scoped_acquire acquire;

        py::module_ detector2d = py::module_::import("reconstruct.detector2d");
        py::object get_detector2d = detector2d.attr("get_detector2d");

        py::module_ detector3d = py::module_::import("reconstruct.detector3d");
        py::object get_detector3d = detector3d.attr("get_detector3d");

        if (!py::isinstance<py::function>(get_detector2d) || 
            !py::isinstance<py::function>(get_detector3d)) {
            throw std::runtime_error("One or both functions are not callable.");
        }

        std::cout << "[Pybind] Detector functions found and valid." << std::endl;
    } catch (const py::error_already_set& e) {
        std::cerr << "[Pybind] Python error: " << e.what() << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "[Pybind] Exception: " << e.what() << std::endl;
        throw;
    }
}



cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{
    if(mSensor!=STEREO && mSensor!=IMU_STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            cout << "Reset stereo..." << endl;
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_STEREO)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, string filename)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }


    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{
    if(mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            cout << "SYSTEM-> Reseting active map in monocular case" << endl;
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}



void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpAtlas->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::ResetActiveMap()
{
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    #ifdef COVINS_MOD
//    #ifndef NO_LOOP_FINDER
    mpLoopCloser->RequestFinish();
//    #endif
    #else
    mpLoopCloser->RequestFinish();
    #endif
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    #ifdef COVINS_MOD
//    #ifndef NO_LOOP_FINDER
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    #else
//    while(!mpLocalMapper->isFinished())
//    #endif
//    #else
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    #endif
    {
        if(!mpLocalMapper->isFinished())
            cout << "mpLocalMapper is not finished" << endl;
        #ifdef COVINS_MOD
//        #ifndef NO_LOOP_FINDER
        if(!mpLoopCloser->isFinished())
            cout << "mpLoopCloser is not finished" << endl;
        if(mpLoopCloser->isRunningGBA()){
            cout << "mpLoopCloser is running GBA" << endl;
            cout << "break anyway..." << endl;
            break;
        }
//        #endif
        #else
        if(!mpLoopCloser->isFinished())
            cout << "mpLoopCloser is not finished" << endl;
        if(mpLoopCloser->isRunningGBA()){
            cout << "mpLoopCloser is running GBA" << endl;
            cout << "break anyway..." << endl;
            break;
        }
        #endif
        usleep(5000);
    }

    #ifdef COVINS_MOD
    comm_->SetFinish();
    while(!comm_->IsFinished()) {
        cout << "comm_ is not finished" << endl;
        usleep(5000);
    }
    #endif

    #ifdef COVINS_MOD
    std::cout << "Joining Threads" << std::endl;
    std::cout << "--> Join Mapping Thread" << std::endl;
    mptLocalMapping->join();
    std::cout << "--> Join LC Thread" << std::endl;
    mptLoopClosing->join();
    std::cout << "--> Join Comm Thread" << std::endl;
    thread_comm_->join();
    if(mpViewer) {
        std::cout << "--> Join Viewer Thread" << std::endl;
        mptViewer->join();
    }
    std::cout << "Done" << std::endl;
    #endif

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");

#ifdef REGISTER_TIMES
    mpTracker->PrintTimeStats();
#endif
}



void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    // cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
}

void System::SaveTrajectoryEuRoC(const string &filename)
{

    cout << endl << "Saving trajectory to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }*/

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            continue;
        }

        Trw = Trw*pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO)
        {
            cv::Mat Tbw = pKF->mImuCalib.Tbc*(*lit)*Trw;
            cv::Mat Rwb = Tbw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twb = -Rwb*Tbw.rowRange(0,3).col(3);
            vector<float> q = Converter::toQuaternion(Rwb);
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
        else
        {
            cv::Mat Tcw = (*lit)*Trw;
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
            vector<float> q = Converter::toQuaternion(Rwc);
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }

    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}


void System::SaveKeyFrameTrajectoryEuRoC(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO)
        {
            cv::Mat R = pKF->GetImuRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat twb = pKF->GetImuPosition();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }
        else
        {
            cv::Mat R = pKF->GetRotation();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
    }
    f.close();
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

double System::GetTimeFromIMUInit()
{
    double aux = mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    if ((aux>0.) && mpAtlas->isImuInitialized())
        return mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    else
        return 0.f;
}

bool System::isLost()
{
    if (!mpAtlas->isImuInitialized())
        return false;
    else
    {
        if ((mpTracker->mState==Tracking::LOST))
            return true;
        else
            return false;
    }
}


bool System::isFinished()
{
    return (GetTimeFromIMUInit()>0.1);
}

void System::ChangeDataset()
{
    if(mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
    {
        mpTracker->ResetActiveMap();
    }
    else
    {
        mpTracker->CreateMapInAtlas();
    }

    mpTracker->NewDataset();
}

#ifdef REGISTER_TIMES
void System::InsertRectTime(double& time)
{
    mpTracker->vdRectStereo_ms.push_back(time);
}

void System::InsertTrackTime(double& time)
{
    mpTracker->vdTrackTotal_ms.push_back(time);
}
#endif


} //namespace ORB_SLAM


