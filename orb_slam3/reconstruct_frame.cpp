/**
* 
* Tariq Abuhashim.
* Adopted from ORBSLAM3 examples.
* Kitti Monocular-Inertial example.
* This reads Kitti images and inertial data using actual time stamps.
* Raw data can be used (previous examples only use odometry data).
* KITTI.yaml has been updated to capture data and Inertial changes.
*
* 21-Oct-2022
*
*/

//#include <stdio.h>
//#include <stdlib.h>
//#include <algorithm>
//#include <fstream>
//#include <ctime>
//#include <sstream>
//#include <vector>

#include <Python.h>

#include <unistd.h>
#include <chrono>
#include <vector>
#include <string>
#include <thread>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "tcp_interface.hpp"

#include <pybind11/eigen.h>
#include <pybind11/embed.h> // Required for embedding Python in C++
#include <pybind11/numpy.h>
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

py::array_t<double> vector3dToNumpyArray(const std::vector<Point>& input) {
    py::array_t<double> result(input.size() * 3); // One 3D point is represented by three double values
    py::buffer_info bufInfo = result.request();
    double* ptr = static_cast<double*>(bufInfo.ptr);
    for (const auto& vec : input) {
        //*ptr++ = vec.x(); // if input is std::vector<Vector3d>
        //*ptr++ = vec.y(); // if input is std::vector<Vector3d>
        //*ptr++ = vec.z(); // if input is std::vector<Vector3d>
        *ptr++ = vec.x;
        *ptr++ = vec.y;
        *ptr++ = vec.z;
    }
	std::vector<size_t> shape = {input.size(), 3};
    result.resize(shape); // Reshape to a 2D array
    return result;
};

py::array_t<unsigned char> cvMatToNumpyArray(const cv::Mat& mat) {
	std::vector<size_t> shape = {static_cast<size_t>(mat.rows), 
								static_cast<size_t>(mat.cols), 
								static_cast<size_t>(mat.channels())};
    py::array_t<unsigned char> result = py::array_t<unsigned char>(shape, mat.data);
    return result;
}

class System 
{

public:

	/* System constructor
	*/
	System (const std::string &strSettingsFile, const std::string &strSequence) {

		//Check settings file
		cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
		if(!fSettings.isOpened())
   	 	{
       		std::cerr << "Failed to open settings file at: " << strSettingsFile << std::endl;
       		exit(-1);
    	}

		// Python interpreter
		std::cout << "Starting Python interpreter ..." << std::endl;
        //if (!Py_IsInitialized())
        py::initialize_interpreter(); // Initialize first
		//py::gil_scoped_acquire acquire; // Then acquire the GIL	
		//py::print(py::module_::import("sys").attr("executable"));
		//py::print(py::module::import("sys").attr("version"));
		//py::print(py::module::import("os").attr("environ"));

        // Load Python modules.
        //try {

			// test code
			std::cout << "Import modules ..." << std::endl;
            //py::module numpy = py::module::import("numpy");
            //py::module scipy = py::module::import("scipy");
			//py::module mmdet3d = py::module::import("mmdet3d");
            // ... load more modules as needed ...
/*
			py::module module = py::module::import("reconstruct.script");  // replace with your module name
          	MyClass = module.attr("MyClass");  // replace with your class name
			obj = MyClass();  // Create an object of MyClass

			std::cout << "Testing ..." << std::endl;
			try {
				py::exec(R"(
        			from Thirdparty.mmdetection3d.mmdet3d.models import build_model
    			)");
			} catch (py::error_already_set& e) {
				std::cerr << "Failed to import mmdet3d: " << e.what() << std::endl;
			}
*/

			// actual code
			std::cout << "Import sys module ..." << std::endl;
			py::module sys = py::module::import("sys");
			if (sys == NULL) { // NEVER accept a pointer back from a function without confirming it's not NULL before attempting to de-refence it
				printf("ERROR importing module 'sys'");
				exit(-1);
			}
			sys.attr("path").attr("append")("./");

			std::cout << "Import utils module ..." << std::endl;
			py::module io_utils = py::module::import("reconstruct.utils");
			if (io_utils == NULL) {
				printf("ERROR importing module 'reconstruct.utils'");
				exit(-1);
			}

			std::cout << "Getting python configs ..." << std::endl;
			std::string pyCfgPath = fSettings["DetectorConfigPath"].string();
			pyCfg = io_utils.attr("get_configs")(pyCfgPath);

			std::cout << "Getting deepsdf decoder ..." << std::endl;    
			pyDecoder = io_utils.attr("get_decoder")(pyCfg);

			std::cout << "Import python data sequence ..." << std::endl;
			pySequence = py::module::import("reconstruct").attr("get_sequence")(strSequence, pyCfg);

//			py::module module = py::module::import("reconstruct");
//			std::cout << "1 ..." << std::endl;
//			MyClass = module.attr("get_sequence");
//			std::cout << "2 ..." << std::endl;
//			pySequence = MyClass(strSequence, pyCfg);

			std::cout << "Done ..." << std::endl;

        //}
        //catch (py::error_already_set const &e) {
        //    std::cout << "Error importing module: " << e.what() << std::endl;
        //}

		std::cout << "Initialise python thread ..." << std::endl;
		if (!PyEval_ThreadsInitialized())
            PyEval_InitThreads();  // Initialize Python's threading support
		PyEval_ReleaseThread(PyThreadState_Get()); // release the Global Interpreter Lock (GIL) allows other threads to run in Python.
	}

	/* process the synchronized data
	*/
	template <typename T1, typename T2>
	void processSyncedData(const SensorData<T1>& data1, const SensorData<T2>& data2) {
    	// get the pointcloud
		std::vector<Point> pointCloud = data1.getData().points;
		// get the image
		Image I = data2.getData();
		cv::Mat image(I.height, I.width, CV_8UC3, I.data.data()); // does not copy the data. If vecData goes out of scope or is modified, the cv::Mat will be affected.
    	std::cout << "Processing data: Lidar time(" << data1.getTimestamp()/1e9 << "), Image time(" << data2.getTimestamp()/1e9 << ")" << std::endl;
		std::cout << "Processing data: Lidar data(" << pointCloud.size() << "), Image data(" << I.height << "x" << I.width << ")" << std::endl;
		// call python function
		call_python_function(image, pointCloud);
	}

	~System() {
        if (Py_IsInitialized())
            py::finalize_interpreter();
    }

	/* python function callback using image and lidar points
	*/
	void call_python_function(const cv::Mat& image, const std::vector<Point>& pointCloud) {
		PyThreadStateLock PyThreadLock;
		//py::gil_scoped_acquire acquire;
		// Convert cv::Mat and std::vector<Eigen::Vector3d> to numpy arrays
		auto np_Image = cvMatToNumpyArray(image);
    	auto np_PointCloud = vector3dToNumpyArray(pointCloud);

		// Call the method with the numpy arrays
    	//obj.attr("my_method")(np_Image, np_PointCloud); // Call the method

		//py::list detections = 
		pySequence.attr("get_frame")(np_Image, np_PointCloud);

    	// Get a reference to the Python function
    	//py::object python_function = pySequence.attr("get_frame");
    	// Call the Python function
    	//python_function(py::none(), np_Image, np_PointCloud);

	}

	/* TODO implement this function
	this is only for an object detection demo, not utilised by object-slam
	*/
	void DetectObjects(const cv::Mat& image, const std::vector<Point>& lidar, const double& tframe) {
		// accept image and lidar frames
		// feed them to python modules
		call_python_function(image, lidar);
		// factor the output into detections
		// send outputs over TCP?
	}

	/* works for a grayscale image
	*/
	py::array_t<unsigned char> mat_to_nparray(cv::Mat &image) {
		/*int rows = 480;   // Height of the image
		int cols = 640;   // Width of the image
		unsigned char* data = ... // pointer to your image data
		cv::Mat image(rows, cols, CV_8UC1, data);  // Create cv::Mat
		*/
		// Convert cv::Mat to py::array
		py::array_t<unsigned char> dst = py::cast(image);
		return dst;
	}

	/* finalise the interpreter
	*/
	void finalize() {
        py::finalize_interpreter();
    }

protected:

    py::object pyCfg;
    py::object pyDecoder;
    py::object pySequence;
	py::object MyClass;
	py::object obj;
};

int main(int argc, char *argv[])
{
	if(argc < 3)
    {
        std::cerr << std::endl << "Usage: ./reconstruct_frame path_to_settings path_to_sequence" << std::endl;
		std::cerr << std::endl << "Examples:" << std::endl;
		std::cerr << std::endl << "./reconstruct_frame ~/data/kitti/2011_09_30/KITTI.yaml ~/data/kitti/2011_09_30/" << std::endl;
        return 1;
    }

	std::cout << std::endl << "-------" << std::endl;
    std::cout.precision(12);

	// Initialise readers
	dataQueue<Cloud> queue1;
	dataQueue<Image> queue2;
	dataQueue<Imu> queue3;
    SensorReader<Cloud> reader1(queue1, "127.0.0.1", 4000, "t,2uw,2ui,uw,3d,3uw,3d"); // t(8-bytes/64-bits), uw(2-bytes), ui(4-bytes), d(8-bytes), s(4-bytes)
    SensorReader<Image> reader2(queue2, "127.0.0.1", 4001, "t,3ui,s[15197952]");
	SensorReader<Imu> reader3(queue3, "127.0.0.1", 4002, "t,3ui,s[15197952]");

    // Start threads to read from the sensors
	std::cout << "\n Start data threads ..." << std::endl;
    std::thread t1(&SensorReader<Cloud>::readFromSocket, &reader1); // fills queue1
    std::thread t2(&SensorReader<Image>::readFromSocket, &reader2); // fills queue2

	// processing thread (syncs data in ldQueue and imQueue and runs python function)
	std::cout << "\n Start process thread ..." << std::endl;
	System SLAM(argv[1], argv[2]);

	return 0;

	double recentTime1=-1, recentTime2=-1;
	std::thread processingThread([&]() {
		//auto func = std::bind(&System::call_python_function, &SLAM, std::placeholders::_1, std::placeholders::_2);
		//cv::Mat image = cv::Mat::zeros(740, 1024, CV_8UC3);
		//std::vector<Eigen::Vector3d> pointCloud(1950, Eigen::Vector3d::Zero());
        while (reader1.is_connected() && reader2.is_connected()) {  // continue as long as both readers are alive

			while (!queue1.empty() && !queue2.empty()) {
				
				std::cout << "queue size: lidar " << queue1.size() << ", vision " << queue2.size() << std::endl;
				//func(image, pointCloud);
				std::lock_guard<std::mutex> guard_cloud(reader1.mtxQueue);
				std::lock_guard<std::mutex> guard_image(reader2.mtxQueue);
				SensorData<Cloud> data1 = queue1.front();
				SensorData<Image> data2 = queue2.front();
				recentTime1 = data1.getTimestamp();
				recentTime2 = data2.getTimestamp();
				if (std::abs(recentTime1/1e+9 - recentTime2/1e+9) < EPSILON) {
					// Timestamps are close enough, process data
					// Bind the function
					// you should directly use the result of std::bind which is a callable object, and can be invoked with the () operator.
					auto func = std::bind(&System::processSyncedData<Cloud, Image>, &SLAM, std::placeholders::_1, std::placeholders::_2);
					// Invoke the function
					func(data1, data2);
					// Remove processed data
					queue1.pop();
					queue2.pop();
				}
				else if (recentTime1/1e+9 < recentTime2/1e+9) {
					// Data from queue1 is too old, discard it
					std::lock_guard<std::mutex> guard_cloud(reader1.mtxQueue);
					queue1.pop();
				}
				else {
					// Data from queue2 is too old, discard it
					std::lock_guard<std::mutex> guard_image(reader2.mtxQueue);
					queue2.pop();
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Sleep to avoid busy waiting
        	}
		
			// Data from queue2 is too old, discard it	
			if (queue1.size() > QUEUE_MAX_SIZE && recentTime2 > 0) {

				while(!queue1.empty()) {
					std::lock_guard<std::mutex> guard_image(reader1.mtxQueue);
					SensorData<Cloud> data1 = queue1.front();
					if (data1.getTimestamp()/1e+9 < recentTime2/1e+9) {
						queue1.pop();
					}
					else {
						break;
					}
				}
			}

			// Data from queue1 is too old, discard it
			if (queue2.size() > QUEUE_MAX_SIZE && recentTime1 > 0) {
				while(!queue2.empty()) {
					std::lock_guard<std::mutex> guard_image(reader2.mtxQueue);
					SensorData<Image> data2 = queue2.front();
					if (data2.getTimestamp()/1e+9 < recentTime1/1e+9) {
						queue2.pop();
					}
					else {
						break;
					}
				}
			}

		}
    });

	// Wait for the thread to finish.
	std::cout << "\n Join the main thread ..." << std::endl;
	processingThread.join();

	std::cout << "\n Finalize the interpreter ..." << std::endl;
	SLAM.finalize();

    return 0;   // Python interpreter shuts down here
}