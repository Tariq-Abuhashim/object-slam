/**
* 
* Tariq Abuhashim.
* Adopted from ORBSLAM3 Kitti Stereo-Inertial examples.
* This reads images, nav data and lidar scans through serial TCP ports.
* KITTI.yaml has been updated to capture data and Inertial changes.
*
* NOTE Test rotations using this calculator https://www.andre-gaschler.com/rotationconverter/
*
* SO3 and Sim3 are terms used in the field of mathematics and computer graphics, particularly in the study of 3D transformations.
* 
*	SO3: This refers to the Special Orthogonal Group of degree 3, denoted as SO(3). 
*		In simpler terms, it represents the group of all 3D rotations. 
*		An SO3 transformation preserves distances (it's an isometry) and it also preserves orientation (it's a proper transformation).
* 
*	Sim3: This is short for Similarity Transformation in 3D. 
*		A Sim3 transformation includes rotations, translations, and uniform scaling. 
*		So, it's an extension of SO3 with an added scale factor.
* 
* To put it in practical terms, if you're transforming a 3D model in a computer program, an SO3 transformation might rotate the model, 
*		whereas a Sim3 transformation could also scale the model larger or smaller, in addition to rotation and translation.
* 
* Updates:
* 21-May-2023
* 21-Jul-2023
*
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>

#include <System.h>
#include "ImuTypes.h"
#include "Optimizer.h"

// COVINS
#include <covins/covins_base/config_comm.hpp> //for covins_params

using namespace std;

// Example function to process synchronized data
void processSyncedData(const CloudData& data1, const ImageData& data2) {
    // Replace this with your actual processing code
    std::cout << "Processing data: Lidar time(" << data1.getTimestamp()/1e9 << "), Image time(" << data2.getTimestamp()/1e9 << ")" << std::endl;
}

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        cerr << endl << "Usage: ./demo path_to_vocabulary path_to_settings path_to_sequence" << endl;
		cerr << endl << "Examples:" << endl;
		cerr << endl << "./mono_snark Vocabulary/ORBvoc.bin configs/vulcan.yaml ~/data/kitti/2011_09_30/" << endl;
        return 1;
    }

	cout << endl << "-------" << endl;
    cout.precision(17);

	// processing thread (syncs data in ldQueue and imQueue and runs python function)
	std::cout << "\n Start process thread ..." << std::endl;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    #ifdef COVINS_MOD
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, covins_params::orb::activate_visualization, 0, argv[3]);
    #else
	ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true, 0, argv[3]);
	#endif

	// open port settings
	cv::FileStorage fSettings("configs/ports.yaml", cv::FileStorage::READ);
	if(!fSettings.isOpened())
	{
		cerr << "Failed to open settings file at: configs/ports.yaml" << endl;
		exit(-1);
	}

	// set where to send the detection results
    // you can listen to this data using  $ nc -l 4003
    // TODO how to relate this connection to serial in COVINS
	SLAM.setSocket(fSettings["host"], fSettings["det2d.port"]); /* ("127.0.0.1", 4003) */

	// Initialise readers
	dataQueue lidarQueue; // for CloudData
	dataQueue imageQueue; // for ImageData
	dataQueue imuQueue;  // for ImuData
    CloudReader lidar_reader(lidarQueue, "127.0.0.1", 4000, "t,2uw,2ui,uw,3d,3uw,3d"); /* ("127.0.0.1", 4000) */
    ImageReader image_reader(imageQueue,"127.0.0.1", 4001, "t,3ui,s[15197952]"); /* ("127.0.0.1", 4001) */
	ImuReader imu_reader(imuQueue, "127.0.0.1", 4002, "t,21f"); /* ("127.0.0.1", 4002) */

	// Start threads to read from the sensors
	std::cout << "\n Start data threads ..." << std::endl;
    std::thread t1(&CloudReader::readFromSocket, &lidar_reader); // fills lidarQueue
    std::thread t2(&ImageReader::readFromSocket, &image_reader); // fills imageQueue
	std::thread t3(&ImuReader::readFromSocket, &imu_reader); // fills imuQueue
	//usleep(1e6); // (not reliable) allow t1 and t2 to start
	std::this_thread::sleep_for(std::chrono::seconds(1)); // ensure that threads have started before moving on

    double lidar_time=-1, image_time=-1, imu_time=-1;
    while (lidar_reader.is_connected() && image_reader.is_connected()) {  // continue as long as both readers are alive

		while (!lidar_reader.safeEmpty() && !image_reader.safeEmpty()) {
			std::shared_ptr<SensorData> lidar_ptr = lidar_reader.safeFront();
			std::shared_ptr<SensorData> image_ptr = image_reader.safeFront();
			std::shared_ptr<CloudData> actual_lidar_ptr = std::dynamic_pointer_cast<CloudData>(lidar_ptr);
			std::shared_ptr<ImageData> actual_image_ptr = std::dynamic_pointer_cast<ImageData>(image_ptr);
			CloudData& lidar = *actual_lidar_ptr; // dereferencing
			lidar_time = lidar.getTimestamp()/1e+9;
			ImageData& image = *actual_image_ptr; // dereferencing
			image_time = image.getTimestamp()/1e+9;

			if (std::abs(lidar_time - image_time) < EPSILON) {
				// get the imu match
				dataQueue relevant_imu_data;
				while (!imu_reader.safeEmpty()) {
					std::shared_ptr<SensorData> imu_ptr = imu_reader.safeFront();
					std::shared_ptr<ImuData> actual_imu_ptr = std::dynamic_pointer_cast<ImuData>(imu_ptr);
					ImuData& imu = *actual_imu_ptr; // dereferencing
					imu_time = imu.getTimestamp()/1e+9;
					if (imu_time <= lidar_time) {
						relevant_imu_data.push_back(imu_ptr);
						imu_reader.safePop();
					}	
					else break;
				}
				
				std::cout << lidar_time << " " << image_time << " " << relevant_imu_data.size() << std::endl;
//				std::cout << lidar_time << " " << image_time << std::endl;

				// Timestamps are close enough, process data
				// you should directly use the result of std::bind which is a callable object, and can be invoked with the () operator.
//				auto func = std::bind(&ORB_SLAM3::System::processSyncedData, &SLAM, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
//				func(lidar, image, relevant_imu_data); // Invoke the function
				lidar_reader.safePop(); // Remove processed data
				image_reader.safePop();
				relevant_imu_data.clear(); // Clear the relevant_imu_data for the next iteration
			}

			else if (lidar_time < image_time) {
				// Data from lidarQueue is too old, discard it
				//std::lock_guard<std::mutex> guard_cloud(reader1.mtxQueue);
				lidar_reader.safePop();
			}

			else {
				// Data from imageQueue is too old, discard it
				//std::lock_guard<std::mutex> guard_image(reader2.mtxQueue);
				image_reader.safePop();
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Sleep to avoid busy waiting
		}

/*
		// Data from lidarQueue is too old, discard it	
		if (lidar_reader.size() > QUEUE_MAX_SIZE && recentTime2 > 0) {

			while(!lidar_reader.safeEmpty()) {
				std::lock_guard<std::mutex> guard_image(reader1->mtxQueue);
				SensorData<Cloud> lidar = lidar_reader.safeFront();
				if (lidar.getTimestamp()/1e+9 < recentTime2/1e+9) {
					lidar_reader.safePop();
				}
				else {
					break;
				}
			}
		}

		// Data from imageQueue is too old, discard it
		if (image_reader.size() > QUEUE_MAX_SIZE && recentTime1 > 0) {
			while(!image_reader.safeEmpty()) {
				std::lock_guard<std::mutex> guard_image(reader2->mtxQueue);
				SensorData<Image> image = image_reader.safeFront();
				if (image.getTimestamp()/1e+9 < recentTime1/1e+9) {
					image_reader.safePop();
				}
				else {
					break;
				}
			}
		}
*/

	}

	std::cout << "\n Joining lidar thread ..." << std::endl;
	t1.join();
	std::cout << "\n Joining camera thread ..." << std::endl;
	t2.join();

    // Stop all threads
	std::cout << "\n Finalize the interpreter ..." << std::endl;
	SLAM.finalize();

	std::cout << "\n Shutdown SLAM ..." << std::endl;
    SLAM.Shutdown();
    return 0;
}
