/**
* 
* Tariq Abuhashim.
* Adopted from ORBSLAM3 examples.
* Kitti Stereo-Inertial example.
* This reads Kitti images and inertial data using actual time stamps.
* Raw data can be used (previous examples only use odometry data).
* KITTI.yaml has been updated to capture data and Inertial changes.
*
* 21-Oct-2022
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

// COVINS
#include <covins/covins_base/config_comm.hpp> //FIXME defines COVINS_MOD

#include <filesystem>
namespace fs = std::filesystem;

using namespace std;

void LoadImages(const string &pathSeq, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps);

void LoadIMU(const string &pathSeq, vector<double> &vTimeStamps, 
				vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);
				
std::vector<std::pair<std::string, std::string>> GetDrivePairs(const std::string &baseDir);

int main(int argc, char **argv)
{

		if(argc < 4)
	{
		std::cerr << std::endl << "Usage: " << std::endl;
		          
		std::cerr << "./stereo_inertial_kitti "
		          << "Vocabulary/ORBvoc.txt "
		          << "<path_to_vocabulary> "
		          << "<path_to_settings> "
		          << "<path_to_seq1> "
		          << "[<path_to_seq2> ...]"
		          << std::endl;

		std::cerr << std::endl << "Examples:" << std::endl;

		std::cerr << "./stereo_inertial_kitti "
		          << "Vocabulary/ORBvoc.txt "
		          << "/media/mrt/Whale/data/kitti/2011_09_30/KITTI.yaml "
		          << "/media/mrt/Whale/data/kitti/2011_09_30/2011_09_30_drive_0018"
		          << std::endl;

		std::cerr << std::endl << "Debug mode (then type <run>):" << std::endl;
		std::cerr << "gdb --args ./stereo_inertial_kitti "
		          << "<path_to_vocabulary> "
		          << "<path_to_settings> "
		          << "<path_to_seq1> "
		          << std::endl;
		          
		std::cerr << std::endl << "Find memory leaks:" << std::endl;
		std::cerr << "valgrind ./stereo_inertial_kitti "
		          << "<path_to_vocabulary> "
		          << "<path_to_settings> "
		          << "<path_to_seq1> "
		          << std::endl;

		return 1;
	}

	const string vocabFile = argv[1];
    const string settingsFile = argv[2];
    
    const int num_seq = argc - 3;
    cout << "[STEREO_INERTIAL_KITTI] num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 1) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "[STEREO_INERTIAL_KITTI] file name: " << file_name << endl;
    }

    /* Load all sequences: */
    int seq;
    // Images
    vector< vector<double> > vTimestampsCam; vTimestampsCam.resize(num_seq);
    vector< vector<string> > vstrImageLeft; vstrImageLeft.resize(num_seq);
    vector< vector<string> > vstrImageRight; vstrImageRight.resize(num_seq);
    vector<int> nImages; nImages.resize(num_seq);
	// Imu
	vector< vector<double> > vTimestampsImu; vTimestampsImu.resize(num_seq);
    vector< vector<cv::Point3f> > vGyro; vGyro.resize(num_seq);
    vector< vector<cv::Point3f> > vAcc; vAcc.resize(num_seq);
    vector<int> nImu; nImu.resize(num_seq);
    vector<int> first_imu(num_seq,0);
    // Sequence by date and drive number
    //cv::FileStorage fSettings(settingsFile, cv::FileStorage::READ);
	//const string day = fSettings["day"];
	//const string seqnum = fSettings["seq"];
	cout << endl << "-------" << endl;
    cout.precision(17);
    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        //string pathSeq(argv[(2*seq) + 3]);
        //string pathTimeStamps(argv[(2*seq) + 4]);
		string pathSeq(argv[(2*seq) + 3]);
		
		//auto [pathImages, pathIMU] = drivePairs.at(0);

        cout << "Loading images for sequence " << seq << "...";
        LoadImages(pathSeq + "_sync", 
        	vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

		cout << "Loading IMU for sequence " << seq << "...";
		LoadIMU(pathSeq + "_extract", 
			vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
		cout << "LOADED!" << endl;

/*		for (int i = 0; i<vTimestampsImu[seq].size(); i++) {
			cout << vTimestampsImu[seq][i] << " " 
				 << vAcc[seq][i] << " " << vGyro[seq][i] << endl;
		}
*/
        nImages[seq] = vstrImageLeft[seq].size();
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();

        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first
        while(first_imu[seq] < vTimestampsImu[seq].size() &&
        	vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered
    }

	cout << endl << "-------" << endl;

    /* Vector for tracking time statistics */

    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);
    double t_track = 0;
    double ttrack_tot = 0;

    /* Create SLAM system. It initializes all system threads and gets ready to process frames.*/

    #ifdef COVINS_MOD
    ORB_SLAM3::System SLAM(vocabFile,
    					settingsFile,
    					ORB_SLAM3::System::IMU_STEREO, 
    					covins_params::orb::activate_visualization, 
    					0, 
    					argv[3]); // FIXME if argv[3] is sequnce path, how to set for (num_seq>1)
    #else
	ORB_SLAM3::System SLAM(vocabFile,
    					settingsFile,
    					ORB_SLAM3::System::IMU_STEREO, 
    					true, 
    					0, 
    					argv[3]); // FIXME if argv[3] is sequnce path, how to set for (num_seq>1)
	#endif

	/* System test code */

	if (false) {
		std::cout << "\n  Runnig a TEST ONLY loop ... \n" << std::endl;
		for (seq = 0; seq<num_seq; seq++)
		{
			std::cout << "\n Starting for sequence " << seq << " ... \n" << std::endl;
			for(int ni=0; ni<nImages[seq]; ni++)
		    {
				cout << "Loading image at: " << string(vstrImageLeft[seq][ni]) << endl;
				usleep(1);
			}
		}
		cv::waitKey(0);
		std::cout << "\n  Shutting down object slam ... \n" << std::endl;
		SLAM.Shutdown();

		std::cout << "\n  Completed ... \n" << std::endl;
		return 0;
	}

	/* Actual code */

    cv::Mat imLeft, imRight;
    for (seq = 0; seq<num_seq; seq++)
    {
		std::cout << "Starting for sequence " << seq << " ... " << std::endl;

        // Seq loop
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        double t_track = 0;
        int proccIm = 0;
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
        
        	cout << ni << endl;
        	cout << vstrImageLeft[seq][ni] << endl;  
        	cout << vstrImageRight[seq][ni] << endl;
        	
            // Read left and right images from file
            imLeft = cv::imread(vstrImageLeft[seq][ni],cv::IMREAD_UNCHANGED);
            imRight = cv::imread(vstrImageRight[seq][ni],cv::IMREAD_UNCHANGED);

            if(imLeft.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageLeft[seq][ni]) << endl;
                return 1;
            }

            if(imRight.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageRight[seq][ni]) << endl;
                return 1;
            }

            double tframe = vTimestampsCam[seq][ni];

            // Load imu measurements from previous frame
            vImuMeas.clear();
            if(ni>0) {
                while(first_imu[seq] < vTimestampsImu[seq].size() &&
      				vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][ni])
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,
                    										vAcc[seq][first_imu[seq]].y,
                    										vAcc[seq][first_imu[seq]].z,
                    										vGyro[seq][first_imu[seq]].x,
                    										vGyro[seq][first_imu[seq]].y,
                    										vGyro[seq][first_imu[seq]].z,
                    										vTimestampsImu[seq][first_imu[seq]]));
                    first_imu[seq]++;
                }
			}

			std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            // Pass the images to the SLAM system
            //SLAM.TrackStereo(imLeft,imRight,tframe,vImuMeas);

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2-t1).count();
            ttrack_tot += t_track;
            vTimesTrack[ni] = t_track;

#ifdef REGISTER_TIMES
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
			ttrack_tot += ttrack;

            vTimesTrack[ni]=ttrack;

			// Wait to load the next frame (following actually timestamps of data)
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];
            if(t_track<T)
                usleep((T-t_track)*1e6); // 1e6
                
        }
        
        if(seq < num_seq - 1)
        {
            cout << "[STEREO_KITTI] Changing the dataset" << endl;
            SLAM.ChangeDataset();
        }
    }
    
    // Wait viewer
    cv::waitKey(0);
    
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &pathSeq, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
	std::cout << "[LoadImages] pathSeq := " + pathSeq << std::endl;
	
	vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    
    /* get timestamps */
    ifstream fTimes;
    //string strPathTimeFile = pathSeq + "/times.txt";
	string strPathTimeFile = pathSeq + "/oxts/timestamps.txt";
    fTimes.open(strPathTimeFile.c_str());
    if (!fTimes.is_open()) {
    	std::cerr << "[LoadImages] Couldn't open " << strPathTimeFile << std::endl;
    	exit(1);
	}
    string str;
    while (getline(fTimes, str))
    {
        if(!str.empty())
        {
			int yyyy, mm, dd, h, m = 0;
			double s = 0;
			if (sscanf(str.c_str(), "%d-%d-%d %d:%d:%lf", &yyyy, &mm, &dd, &h, &m, &s) == 6)
			{
  				double t = h *3600 + m*60 + s;
				//cout << t << endl;
				vTimeStamps.push_back(t);
			}
        }
    }
    const int nTimes = vTimeStamps.size();
    std::cout << "[LoadImages] nTimes " << nTimes << "\n";
    
    /* get image names as per timestamps */
    string strPrefixLeft = pathSeq + "/image_00/data/";
    string strPrefixRight = pathSeq + "/image_01/data/";
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);
    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(10) << i; // FIXME 6 or 10 depending on dataset
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

void LoadIMU(const string &pathSeq, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{

	std::cout << "[LoadIMU] pathSeq := " + pathSeq << std::endl;
	
	vAcc.reserve(50000);
    vGyro.reserve(50000);
	
	/* get timestamps */
    ifstream fTimes;
    //string strPathTimeFile = pathSeq + "/times.txt";
	string strPathTimeFile = pathSeq + "/oxts/timestamps.txt";
    fTimes.open(strPathTimeFile.c_str());
    if (!fTimes.is_open()) {
    	std::cerr << "[LoadImages] Couldn't open " << strPathTimeFile << std::endl;
    	exit(1);
	}
	string str;
    while (getline(fTimes, str))
    {
        if(!str.empty())
        {
			int yyyy, mm, dd, h, m = 0;
			double s = 0;
			if (sscanf(str.c_str(), "%d-%d-%d %d:%d:%lf", &yyyy, &mm, &dd, &h, &m, &s) == 6)
			{
  				double t = h *3600 + m*60 + s;
				//cout << t << endl;
				vTimeStamps.push_back(t);
			}
        }
    }
	const int nTimes = vTimeStamps.size();
	std::cout << "[LoadImages] nTimes " << nTimes << "\n";

	/* get Imu data as per timestamps */
	for(int i=0; i<nTimes; i++)
	{
		stringstream ss;
		ss << setfill('0') << setw(10) << i;
		string strImuPath = pathSeq + "/oxts/data/" + ss.str() + ".txt";
		//cout << strImuPath << endl;
		ifstream fImu;
		fImu.open(strImuPath.c_str());

		string s;
		getline(fImu,s);
		if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[30];
            int count = 0;
            while ((pos = s.find(' ')) != string::npos) {
                item = s.substr(0, pos); // get sub string at starting location 0 with length pos
                data[count++] = stod(item); // Convert string to double
                s.erase(0, pos + 1); // string& erase (size_t pos = 0, size_t len = npos);
            }
            item = s.substr(0, pos);
            data[29] = stod(item);

            //vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[14],data[15],data[16]));
            vGyro.push_back(cv::Point3f(data[20],data[21],data[22]));
        }
	}
}
