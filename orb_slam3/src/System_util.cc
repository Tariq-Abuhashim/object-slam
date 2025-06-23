
/**
* updated - Dec, 2022
**/

#include "System.h"

#include <Eigen/Geometry>  // For transformations in vector3dToNumpyArray
#include <Eigen/Dense> //

long long lidar_index = 0;
double trans_roll_  = 0.0;
double trans_pitch_ = 0.0;
double trans_yaw_   = 0.0;
double trans_tx_    = 0.0;
double trans_ty_    = 0.0;
double trans_tz_    = 0.0;
Eigen::Affine3f transform_matrix_ = Eigen::Affine3f::Identity();

static std::vector<std::string> file_lists;

namespace ORB_SLAM3
{

	void System::setSocket(const std::string& server, const std::string& port) {
		cout << "Initialise boost socket to send detections ..." << endl;
		// Resolve the server address and port
		boost::asio::io_service io_service;
		socket_ =  new boost::asio::ip::tcp::socket(io_service);
		boost::asio::ip::tcp::resolver resolver(io_service);
		boost::asio::ip::tcp::resolver::query query(server, port);
		boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
		// Try to connect to the server
		boost::asio::ip::tcp::resolver::iterator end;
	   	boost::system::error_code error = boost::asio::error::host_not_found;
		while (endpoint_iterator != end && error) {
			socket_->close();
			socket_->connect(*endpoint_iterator++, error);
		}
		if (error) {
			throw boost::system::system_error(error);
		}
	};

	// Example function to process synchronized data
	//void System::processSyncedData(const CloudData& data1, const ImageData& data2) {
	//	// Replace this with your actual processing code
	//	std::cout << "Processing data: Lidar time(" << data1.getTimestamp()/1e9 << "), Image time(" << data2.getTimestamp()/1e9 << ")" << std::endl;
	//}

	/* a strictly CloudData and ImageData function */
	void System::processSyncedData(const CloudData& lidar, const ImageData& image, dataQueue& imuQueue) {
		// get imu measurement
		double lidarTime = lidar.getTimestamp()/1e+9;
		double imuTime = -1;
		ImuData* imu = nullptr;
		while (!imuQueue.empty()) {
		//for (auto& imu_ptr : imuQueue) {
			//std::lock_guard<std::mutex> guard_cloud(reader1.mtxQueue);
			std::shared_ptr<SensorData> imu_ptr = imuQueue.front();
			std::shared_ptr<ImuData> actual_imu_ptr = std::dynamic_pointer_cast<ImuData>(imu_ptr);
			if (actual_imu_ptr) {
				imuTime = actual_imu_ptr->getTimestamp()/1e+9;

				std::cout << lidarTime << " " << imuTime << std::endl;

				if (std::abs(imuTime-lidarTime) < EPSILON) {
					imu = actual_imu_ptr.get(); // get raw pointer
					break;
				}
			}
			imuQueue.pop_front();  // Pop the checked IMU data from the front
		}
		if (!imu) {
			std::cerr << "No Imu match" << std::endl;
			return;
		}
    	// get the pointcloud
		std::vector<Point> pointCloud = lidar.getData().points;
		/*
			Add code here to align the point cloud using imu orientation and height over ground
		*/
		// get the image
		Image I = image.getData();
		cv::Mat Cvimage(I.height, I.width, CV_8UC3, I.data.data()); // does not copy the data. If vecData goes out of scope or is modified, the cv::Mat will be affected.
    	//std::cout << "Processing data: Lidar time(" << data1.getTimestamp()/1e9 << "), Image time(" << data2.getTimestamp()/1e9 << ")" << std::endl;
		//std::cout << "Processing data: Lidar data(" << pointCloud.size() << "), Image data(" << I.height << "x" << I.width << ")" << std::endl;
		// call python function
		call_python_function(image.getTimestamp(), Cvimage, pointCloud, imu->getData());
	};

	// increments file name counter
	void increment(){
		++lidar_index;
	}

	// saves lidar data to pcd file
	void writeToPCD(const std::vector<Point>& input)
	{
		// Open file for writing
		char s[200];
		sprintf(s, "/home/mrt/data/mission_systems/output/pcd/%010lld.pcd", lidar_index); 
		std::ofstream file(s);
		if (!file) {
        	std::cerr << "Could not open file for writing\n";
    	}

		// Write the header
		file << "# .PCD v.7 - Point Cloud Data file format\n"
		     << "VERSION .7\n"
		     << "FIELDS x y z intensity\n"   // Add 'intensity' to FIELDS
		     << "SIZE 4 4 4 4\n"             // Add '4' to SIZE (assuming float intensity)
		     << "TYPE F F F F\n"             // Add 'F' to TYPE (assuming float intensity)
		     << "COUNT 1 1 1 1\n"            // Add '1' to COUNT
		     << "WIDTH " << input.size() << "\n"
		     << "HEIGHT 1\n"
		     << "VIEWPOINT 0 0 0 1 0 0 0\n"
		     << "POINTS " << input.size() << "\n"
		     << "DATA ascii\n";

		// get the max value
		float maxIntensity = -1;
		for (const auto& vec : input) {
			if (static_cast<float>(vec.intensity) > maxIntensity && vec.range > 2.0f) // TODO determine this minimum range
				maxIntensity = static_cast<float>(vec.intensity);
		}
		// Write the point data
		for (const auto& vec : input) {
			if (vec.range < 2.0f) continue; // TODO determine this minimum range
			float x = static_cast<float>(vec.x); //-(0*static_cast<float>(vec.x) + 0*static_cast<float>(vec.y) + 1*static_cast<float>(vec.z) + 0.25);
			float y = static_cast<float>(vec.y); //-(0*static_cast<float>(vec.x) - 1*static_cast<float>(vec.y) + 0*static_cast<float>(vec.z) + 0.10);
			float z = static_cast<float>(vec.z); // (1*static_cast<float>(vec.x) + 0*static_cast<float>(vec.y) + 0*static_cast<float>(vec.z) + 0.00);
			float i = (static_cast<float>(vec.intensity)/maxIntensity); // when using signal
			//float i =  (static_cast<float>(vec.intensity)/255.0f); // when using reflectivity
			//TODO there might be a geometric transformation here
			file << x << " " << y << " " << z << " " << i << "\n";
		}

		file.close();
	}

	// saves lidar data to bin file
	void writeToBIN(const std::vector<Point>& input) { 
		// Open file for writing
		char s[200];
		sprintf(s, "/home/mrt/data/mission_systems/output/bin/%010lld.bin", lidar_index); 
		std::ofstream bin_file(s, ios::out|ios::binary|ios::app);
		if (!bin_file.good()) {
        	std::cerr << "Couldn't open " << s << std::endl;
    	}
		// get the max value
		float maxIntensity = -1;
		for (const auto& vec : input) {
			if (static_cast<float>(vec.intensity) > maxIntensity && vec.range > 2.0f) // TODO determine this minimum range
				maxIntensity = static_cast<float>(vec.intensity);
		}
		std::cout << "maxIntensity=" << maxIntensity << std::endl;
		// Coordinate transformation and filling the file with data
		for (const auto& vec : input) {
			if (vec.range < 2.0f) continue; // TODO determine this minimum range
			float x = static_cast<float>(vec.x);
			float y = static_cast<float>(vec.y);
			float z = static_cast<float>(vec.z);
			//float x_ = ( 0*x + 0*y - 1*z + 0.25);
			//float y_ = (-1*x + 0*y + 0*z + 0.10);
			//float z_ = ( 0*x + 1*y + 0*z + 0.00);
			float x_ = ( 1*x + 0*y + 0*z + 0.18016);
			float y_ = ( 0*x + 1*y + 0*z + 0.03540);
			float z_ = ( 0*x + 0*y + 1*z + 0.10900);
			float i_ =  (static_cast<float>(vec.intensity)/maxIntensity);
			//float i_ =  (static_cast<float>(vec.intensity)/255.0f);
			bin_file.write((char*)&x_, sizeof(float));
			bin_file.write((char*)&y_, sizeof(float));
			bin_file.write((char*)&z_, sizeof(float));
    		bin_file.write((char*)&i_, sizeof(float));
		}
	}

	// saves image data to png file
	void writeToPNG(const cv::Mat& mat)
	{
		char s[200];
		sprintf(s, "/home/mrt/data/mission_systems/output/png/%010lld.png", lidar_index);
		imwrite(s, mat);
	}

	void writeToNAV(const Imu& imu)
	{
		char s[200];
		sprintf(s, "/home/mrt/data/mission_systems/output/nav/%010lld.txt", lidar_index);
		std::ofstream file(s);
		if (!file) {
        	std::cerr << "Could not open file for writing\n";
    	}
		file<< setprecision(9) << 0.0 << " " << 0.0 << " " << 0.0 << " " 
			<< imu.orientation[0] << " " << imu.orientation[1] << " " << imu.orientation[2] << " " 
			<< 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " 
			<< imu.accelerometer[0] << " " << imu.accelerometer[1] << " " << imu.accelerometer[2] << " " 
			<< 0.0 << " " << 0.0 << " " << 0.0 << " " 
			<< imu.gyroscope[0] << " " << imu.gyroscope[1] << " " << imu.gyroscope[2] << " " << 0.0 << " " 
			<< 0.0 << " " << " " << 0.0 << " " << 0.0 << endl;
		file.close();
	}

	// converts lidar data to numpy array
	py::array_t<float> vector3dToNumpyArray(const std::vector<Point>& input, const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
		py::array_t<float> result(input.size() * 4); // One 3D point is represented by three double values + intensity
		py::buffer_info bufInfo = result.request();
		float* ptr = static_cast<float*>(bufInfo.ptr);
		// get the max reflectivity
		float maxIntensity = -1;
		for (const auto& vec : input) {
			if (static_cast<float>(vec.intensity) > maxIntensity)
				maxIntensity = static_cast<float>(vec.intensity);
		}
		// Apply rotation and translation and store in py::array_t
		for (const auto& vec : input) {
			Eigen::Vector3d point(vec.x, vec.y, vec.z);
			Eigen::Vector3d transformedPoint = R * point + t;
		    *ptr++ = static_cast<float>(transformedPoint.x());
		    *ptr++ = static_cast<float>(transformedPoint.y());
		    *ptr++ = static_cast<float>(transformedPoint.z());
			*ptr++ = static_cast<float>(vec.intensity)/maxIntensity;
		}
		std::cout << "maxIntensity=" << maxIntensity << std::endl;
		std::vector<size_t> shape = {input.size(), 4};
		result.resize(shape); // Reshape to a 2D array
		return result;
	};

	// converts image data to numpy array
	py::array_t<unsigned char> cvMatToNumpyArray(const cv::Mat& mat) {
		std::vector<size_t> shape = {static_cast<size_t>(mat.rows), 
									static_cast<size_t>(mat.cols), 
									static_cast<size_t>(mat.channels())};
		py::array_t<unsigned char> result = py::array_t<unsigned char>(shape, mat.data);
		return result;
	}

	// python function callback using image and lidar points
	void System::call_python_function(const double& Time, const cv::Mat& image, const std::vector<Point>& pointCloud, 
										const Imu& imu) {
		PyThreadStateLock PyThreadLock;
		//py::gil_scoped_acquire acquire;

		// Convert cv::Mat and std::vector<Eigen::Vector3d> to numpy arrays
		auto np_Image = cvMatToNumpyArray(image);

		std::cout << imu.orientation[0] << " " << imu.orientation[1] << " " << imu.orientation[2] << std::endl;

		// rotation angles (for mission systems)
		double r = 0;//imu.orientation[0] * M_PI/180.0;  // rotation around x-axis
		double p = 0;//imu.orientation[1] * M_PI/180.0;  // rotation around y-axis
		double y = 0;//imu.orientation[2] * M_PI/180.0;  // rotation around z-axis

		// construct rotation matrix using a roll-pitch-yaw convention (order in which you multiply the Eigen::AngleAxisd instances matters)
		Eigen::Matrix3d R;
		R = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ());
		Eigen::Vector3d t(0.0, 0.0, 0.0);
    	auto np_PointCloud = vector3dToNumpyArray(pointCloud, R, t);

		// Serialise data to the hard-drive
		writeToPCD(pointCloud);
		//writeToBIN(pointCloud);
		writeToPNG(image);
		writeToNAV(imu);
		increment(); // moves the file name counter
		return;

		// Call the method with the numpy arrays
    	//obj.attr("my_method")(np_Image, np_PointCloud); // Call the method
		py::list detections = pySequence.attr("get_frame")(np_Image, np_PointCloud);

		if (detections.size()<1) return;

		std::stringstream ss;
        ss << Time  << "," << detections.size(); // Comma Se
		for (auto det : detections) {
			//auto mask = det.attr("mask").cast<Eigen::Vector3f>();
			auto bbox = det.attr("bbox").cast<Eigen::Vector4f>();
			auto label = det.attr("label").cast<int>();
			auto score = det.attr("score").cast<float>();
			if (score > 0.85f) {
				// Convert bbox, label, and score to string format and send to socket
                ss << "," << label << "," << bbox.transpose() << "," << score;
			}
		}
		ss << "\n";  // Add newline to separate detections

		if (socket_->is_open()) {
			std::string data_to_send = ss.str();   
		    boost::system::error_code error;
			boost::asio::write(*socket_, boost::asio::buffer(data_to_send), boost::asio::transfer_all(), error);
			if (error) {
				throw boost::system::system_error(error);
			}
		}
		else {
			std::cerr << "call_python_function: Socket_ is not open" << std::endl;
		}
			
/*
		for (auto det : detections) {
		    auto pts = det.attr("surface_points").cast<Eigen::MatrixXf>();
		    auto Sim3Tco = det.attr("T_cam_obj").cast<Eigen::Matrix4f>();
		    auto rays = det.attr("rays");
			auto box = det.attr("scale").cast<Eigen::Vector3f>();; // TODO new update, use point-pillar bounding box

		    Eigen::MatrixXf rays_mat;
		    Eigen::VectorXf depth;

		    if (rays.is_none()) {
				continue; // FIXME I updated with this return
		        //std::cout << "No 2D masks associated!" << std::endl;
		        rays_mat = Eigen::Matrix<float, 0, 0>::Zero();
		        depth = Eigen::VectorXf::Zero(0); // DSP changes, this was Eigen::Vector<float, 0>::Zero() 
		    } else {
		        rays_mat = rays.cast<Eigen::MatrixXf>();
		        depth = det.attr("depth").cast<Eigen::VectorXf>();
				count++;
		    }
		    // Create C++ detection instance
		    auto o = new ObjectDetection(Sim3Tco, pts, rays_mat, depth, box);
		    pKF->mvpDetectedObjects.push_back(o);
		}
		pKF->nObj = pKF->mvpDetectedObjects.size();
		pKF->mvpMapObjects = vector<MapObject *>(pKF->nObj, static_cast<MapObject *>(NULL));
		cout << "TRACK_UTIL: KF" << pKF->mnId << " detections: " << pKF->nObj << " 3D cuboids + " << count << " 2D masks." << endl;
*/
    	// Get a reference to the Python function
    	//py::object python_function = pySequence.attr("get_frame");
    	// Call the Python function
    	//python_function(py::none(), np_Image, np_PointCloud);

	}

	void System::SaveMapCurrentFrame(const string &dir, int frameId) {
		stringstream ss;
		ss << setfill('0') << setw(6) << frameId;

		string fname_pts = dir + "/" + ss.str() + "-MapPoints.txt";
		ofstream f_pts;
		f_pts.open(fname_pts.c_str());
		f_pts << fixed;

		Map* mpMap = mpAtlas->GetCurrentMap();

		const vector<MapPoint *> &vpMPs = mpMap->GetAllMapPoints();
		for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
		    if (vpMPs[i]->isBad())
		        continue;
		    cv::Mat pos = vpMPs[i]->GetWorldPos();
		    f_pts << setprecision(9) << pos.at<float>(0) << " " << pos.at<float>(1) << " " << pos.at<float>(2) << endl;
		}
		f_pts.close();

		string fname_objects = dir + "/" + ss.str() + "-MapObjects.txt";
		ofstream f_obj;
		f_obj.open(fname_objects.c_str());
		f_obj << fixed;
		auto mvpMapObjects = mpMap->GetAllMapObjects();
		sort(mvpMapObjects.begin(), mvpMapObjects.end(), MapObject::lId);
		for (MapObject *pMO : mvpMapObjects) {
		    if (!pMO)
		        continue;
		    if (pMO->isBad())
		        continue;
		    if (pMO->mRenderId < 0)
		        continue;
		    if (pMO->isDynamic())
		        continue;

		    f_obj << pMO->mnId << endl;
		    auto Two = pMO->GetPoseSim3();
		    f_obj << setprecision(9) << Two(0, 0) << " " << Two(0, 1) << " " << Two(0, 2) << " " << Two(0, 3) << " " <<
		          Two(1, 0) << " " << Two(1, 1) << " " << Two(1, 2) << " " << Two(1, 3) << " " <<
		          Two(2, 0) << " " << Two(2, 1) << " " << Two(2, 2) << " " << Two(2, 3) << endl;
		    f_obj << setprecision(9) << pMO->GetShapeCode().transpose() << endl;
		}
		f_obj.close();

		vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
		string fname_keyframes = dir + "/" + ss.str() + "-KeyFrames.txt";
		ofstream f_kfs;
		f_kfs.open(fname_keyframes.c_str());
		f_kfs << fixed;
		for (size_t i = 0; i < vpKFs.size(); i++) {
		    KeyFrame *pKF = vpKFs[i];

		    if (pKF->isBad())
		        continue;

		    cv::Mat Tcw = pKF->GetPose();
		    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		    cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
		    f_kfs << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2)
		          << " " << twc.at<float>(0) << " " <<
		          Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " "
		          << twc.at<float>(1) << " " <<
		          Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " "
		          << twc.at<float>(2) << endl;

		}
		f_kfs.close();

		string fname_frame = dir + "/" + ss.str() + "-Camera.txt";
		ofstream f_camera;
		f_camera.open(fname_frame.c_str());
		f_camera << fixed;
		cv::Mat Tcw = mpTracker->mCurrentFrame.mTcw;
		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
		f_camera << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2)
		         << " " << twc.at<float>(0) << " " <<
		         Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " "
		         << twc.at<float>(1) << " " <<
		         Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " "
		         << twc.at<float>(2) << endl;
		f_camera.close();

		cv::Mat frame = mpViewer->GetFrame();
		cv::imwrite(dir + "/" + ss.str() + "-Frame.png", frame);
	}

	void System::SaveEntireMap(const string &dir) {
		string fname_pts = dir + "/MapPoints.txt";
		ofstream f_pts;
		f_pts.open(fname_pts.c_str());
		f_pts << fixed;

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

		const vector<MapPoint *> &vpMPs = pBiggerMap->GetAllMapPoints();
		for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
		    if (vpMPs[i]->isBad())
		        continue;
		    cv::Mat pos = vpMPs[i]->GetWorldPos();
		    f_pts << setprecision(9) << pos.at<float>(0) << " " << pos.at<float>(1) << " " << pos.at<float>(2) << endl;
		}

		string fname_objects = dir + "/MapObjects.txt";
		ofstream f_obj;
		f_obj.open(fname_objects.c_str());
		f_obj << fixed;
		auto mvpMapObjects = pBiggerMap->GetAllMapObjects();
		sort(mvpMapObjects.begin(), mvpMapObjects.end(), MapObject::lId);
		for (MapObject *pMO : mvpMapObjects) {
		    if (!pMO)
		        continue;
		    if (pMO->isBad())
		        continue;
		    if (pMO->GetRenderId() < 0)
		        continue;
		    if (pMO->isDynamic())
		        continue;

		    f_obj << pMO->mnId << endl;
		    auto Two = pMO->GetPoseSim3();
		    f_obj << setprecision(9) << Two(0, 0) << " " << Two(0, 1) << " " << Two(0, 2) << " " << Two(0, 3) << " " <<
		          Two(1, 0) << " " << Two(1, 1) << " " << Two(1, 2) << " " << Two(1, 3) << " " <<
		          Two(2, 0) << " " << Two(2, 1) << " " << Two(2, 2) << " " << Two(2, 3) << endl;
		    f_obj << setprecision(9) << pMO->GetShapeCode().transpose() << endl;
		}
		f_obj.close();

		SaveTrajectoryKITTI(dir + "/Cameras.txt");
	}

}
