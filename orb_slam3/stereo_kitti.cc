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
#include <fstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <System.h>
#include "ImuTypes.h"

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

		std::cerr << "./stereo_kitti "
		          << "Vocabulary/ORBvoc.txt "
		          << "/media/mrt/Whale/data/kitti/07/KITTI04-12.yaml "
		          << "/media/mrt/Whale/data/kitti/07/"
		          << std::endl;

		std::cerr << "./stereo_kitti "
		          << "Vocabulary/ORBvoc.txt "
		          << "/media/mrt/Whale/data/kitti/2011_09_30/KITTI.yaml "
		          << "/media/mrt/Whale/data/kitti/2011_09_30/2011_09_30_drive_0018_sync"
		          << std::endl;

		std::cerr << std::endl << "Debug mode (then type <run>, and backtrace <bt>):" << std::endl;
		std::cerr << "cmake -DCMAKE_BUILD_TYPE=Debug .. \n";
		std::cerr << "make -j  \n";
		std::cerr << "gdb --args ./stereo_kitti "
		          << "<path_to_vocabulary> "
		          << "<path_to_settings> "
		          << "<path_to_seq1> "
		          << std::endl;
		          
		std::cerr << std::endl << "Find memory leaks:" << std::endl;
		std::cerr << "valgrind ./stereo_kitti "
		          << "<path_to_vocabulary> "
		          << "<path_to_settings> "
		          << "<path_to_seq1> "
		          << std::endl;

		return 1;
	}

	const string vocabFile = argv[1];
    const string settingsFile = argv[2];
    
    const int num_seq = argc - 3;
    cout << "[STEREO_KITTI] num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 1) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "[STEREO_KITTI] file name: " << file_name << endl;
    }

    /* Load all sequences: */
    int seq;
    // Images
    vector< vector<double> > vTimestampsCam; vTimestampsCam.resize(num_seq);
    vector< vector<string> > vstrImageLeft; vstrImageLeft.resize(num_seq);
    vector< vector<string> > vstrImageRight; vstrImageRight.resize(num_seq);
    vector<int> nImages; nImages.resize(num_seq);
	cout << endl << "-------" << endl;
    cout.precision(17);
    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        string pathSeq(argv[(2*seq) + 3]);

        cout << "[STEREO_KITTI] Loading images for sequence " << seq << "...\n";
        LoadImages(pathSeq, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
        std::cout 	<< "[STEREO_KITTI] Sequence has " 
        			<< vstrImageLeft[seq].size() 
        			<< " Images ..." << std::endl;

        nImages[seq] = vstrImageLeft[seq].size();
        tot_images += nImages[seq];
    }
	cout << endl << "-------" << endl;
	
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM3::System SLAM(vocabFile, 
						settingsFile, 
						ORB_SLAM3::System::STEREO, 
						true, 
						0, 
						argv[3]); // FIXME if argv[3] is sequnce path, how to set for (num_seq>1)

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);
    double t_track = 0;
    double ttrack_tot = 0;

    cv::Mat imLeft, imRight;
    for (seq = 0; seq<num_seq; seq++)
    {
    	std::cout << std::endl;
		std::cout << "[STEREO_KITTI] Starting for sequence " << seq << " ..." << std::endl;

        // Seq loop
        for(int ni=0; ni<nImages[seq]; ni++)
		//for(int ni=0; ni<1; ni++)
        {
        	//cout << ni << endl;
        	//cout << vstrImageLeft[seq][ni] << endl;  
        	//cout << vstrImageRight[seq][ni] << endl;
        	  	
            // Read left and right images from file
            imLeft = cv::imread(vstrImageLeft[seq][ni],cv::IMREAD_UNCHANGED);
            imRight = cv::imread(vstrImageRight[seq][ni],cv::IMREAD_UNCHANGED);

            if(imLeft.empty())
            {
                cerr << endl << "[STEREO_KITTI] Failed to load image at: "
                     << string(vstrImageLeft[seq][ni]) << endl;
                return 1;
            }

            if(imRight.empty())
            {
                cerr << endl << "[STEREO_KITTI] Failed to load image at: "
                     << string(vstrImageRight[seq][ni]) << endl;
                return 1;
            }
            
            double tframe = vTimestampsCam[seq][ni];
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            
            // Pass the images to the SLAM system
            SLAM.TrackStereo(imLeft,imRight,tframe);
            
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2-t1).count();
            ttrack_tot += t_track;
            vTimesTrack[ni] = t_track;
            
            std::cout << "fps = " << 1000/t_track << std::endl;
            
#ifdef REGISTER_TIMES
            //SLAM.InsertTrackTime(t_track);
#endif
          	usleep(1000.0);
          	
			// Wait to load the next frame (following actually timestamps of data)
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];
            if(t_track<T)
                usleep((T-t_track)*1e6); // 1e6
            
		}
		
		if(seq < num_seq - 1) {
			cout << "[STEREO_KITTI] Changing the dataset" << endl;
			//SLAM.ChangeDataset();
		}
    }
    
    // Wait viewer
    //cv::waitKey(0);
    
    // Stop all threads
    SLAM.Shutdown();
    
    // Save camera trajectory
    /*
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
    */

    return 0;
}

void LoadImages(const string &pathSeq, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{

	std::cout << "[LoadImages] pathSeq := " + pathSeq << std::endl;
	
    ifstream fTimes;
    //string strPathTimeFile = pathSeq + "/times.txt";
	string strPathTimeFile = pathSeq + "/oxts/timestamps.txt";
    fTimes.open(strPathTimeFile.c_str());
    if (!fTimes.is_open()) {
    	std::cerr << "[LoadImages] Couldn't open " << strPathTimeFile << std::endl;
    	exit(1);
	}

	vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    
    /* read timestamps from file
    	format = yyyy-mm-dd hh:mm:s */
    //while(!fTimes.eof()) //eof() only becomes true after a failed read.
/*    string str;
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
    } */
    
    /* read timestamps from file
    	format = seconds */
/*
    string str;
    while (getline(fTimes, str))
    {
		if(!str.empty())
        {
			double s = 0;
			if (sscanf(str.c_str(), "%lf", &s) == 1)
			{
				vTimeStamps.push_back(s);
			}
        }
    } */
    
    /* fake timestamps using fps
    	this is not useful if IMU-Image synchronisation is needed
    */
    const float fps = 10.;
    const float dt = 1. / fps;
    float t = 0.;
    string s;
    while(getline(fTimes,s))
    {
        if(!s.empty())
        {
            vTimeStamps.push_back(t);
            t += dt;
        }
    }
    
	const int nTimes = vTimeStamps.size();
	std::cout << "[LoadImages] nTimes " << nTimes << "\n";
	
	/* Assuming timestamps and images are synchronised */
    string strPrefixLeft = pathSeq + "/image_00/data/";
    string strPrefixRight = pathSeq + "/image_01/data/";
    //string strPrefixLeft = pathSeq + "/image_0/";
    //string strPrefixRight = pathSeq + "/image_1/";
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);
    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(10) << i;  // FIXME 6 or 10
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
