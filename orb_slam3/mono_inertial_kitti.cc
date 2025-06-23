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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>

#include<opencv2/core/core.hpp>

#include <System.h>
#include "ImuTypes.h"
#include "Optimizer.h"

#include "tcp_interface.hpp"

// COVINS
#include <covins/covins_base/config_comm.hpp> //for covins_params

using namespace std;

std::uint64_t to_microseconds(const boost::posix_time::ptime& t) {
    static boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
    return (t - epoch).total_microseconds();
}

void LoadImages(const string &pathSeq, vector<string> &vstrImageLeft,
                vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

double ttrack_tot = 0;
int main(int argc, char *argv[])
{
    if(argc < 4)
    {
        cerr << endl << "Usage: ./demo path_to_vocabulary path_to_settings path_to_sequence" << endl;
		cerr << endl << "Examples:" << endl;
		cerr << endl << "./mono_inertial_kitti Vocabulary/ORBvoc.bin ~/data/kitti/2011_09_30/KITTI.yaml ~/data/kitti/2011_09_30/" << endl;
        return 1;
    }

    const int num_seq = (argc-3)/1; // was 2 instead of 1
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 1) == 1); // was % 2 instead of % 1
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

	cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
	string day = fSettings["day"];
	string seqnum = fSettings["seq"];

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageLeft;
    //vector< vector<string> > vstrImageRight;
    vector< vector<double> > vTimestampsCam;
    vector< vector<cv::Point3f> > vAcc, vGyro;
    vector< vector<double> > vTimestampsImu;
    vector<int> nImages;
    vector<int> nImu;
    vector<int> first_imu(num_seq,0);

    vstrImageLeft.resize(num_seq);
    //vstrImageRight.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

	cout << endl << "-------" << endl;
    cout.precision(17);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        //string pathSeq(argv[(2*seq) + 3]);
        //string pathTimeStamps(argv[(2*seq) + 4]);
		string pathSeq(argv[3]);

        cout << "Loading images for sequence " << seq << "...";
        LoadImages(pathSeq+"/2011_09_"+day+"_drive_00"+seqnum+"_sync", vstrImageLeft[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        cout << "Loading IMU for sequence " << seq << "...";
		LoadIMU(pathSeq+"/2011_09_"+day+"_drive_00"+seqnum+"_extract", vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
		cout << "LOADED!" << endl;

		//for (int i = 0; i<vTimestampsImu[seq].size(); i++)
		//for (int i = 0; i<10; i++)
		//	cout << vTimestampsImu[seq][i] << " " << vAcc[seq][i] << " " << vGyro[seq][i] << endl;

        nImages[seq] = vstrImageLeft[seq].size();
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();

        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first
        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    #ifdef COVINS_MOD
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, covins_params::orb::activate_visualization, 0, argv[3]);
    #else
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, true, 0, argv[3]);
    #endif

	cv::Mat imLeft, imRight;
    for (seq = 0; seq<num_seq; seq++)
    {
		std::cout << "\n Starting for sequence " << seq << " ... \n" << std::endl;

		// Seq loop
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
		double t_track = 0;
        int proccIm = 0;
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
            // Read image from file
            imLeft = cv::imread(vstrImageLeft[seq][ni],cv::IMREAD_UNCHANGED);	
			//imRight = cv::imread(vstrImageRight[seq][ni],cv::IMREAD_UNCHANGED);

            if(imLeft.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageLeft[seq][ni] << endl;
                return 1;
            }

	        //if(imRight.empty())
            //{
            //    cerr << endl << "Failed to load image at: "
            //         << string(vstrImageRight[seq][ni]) << endl;
            //    return 1;
            //}

            double tframe = vTimestampsCam[seq][ni];

            // Load imu measurements from previous frame
            vImuMeas.clear();

            if(ni>0)
                while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni])
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]]));
                    first_imu[seq]++;
                }

    //#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //#else
   //         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    //#endif

            // Pass the image to the SLAM system
            SLAM.TrackMonocular(imLeft,tframe,vImuMeas);

    //#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
   // #else
   //         std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    //#endif

#ifdef REGISTER_TIMES
            double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            ttrack_tot += ttrack;

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }
        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }
    }
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

void LoadImages(const string &pathSeq, vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    //string strPathTimeFile = pathSeq + "/times.txt";
	string strPathTimeFile = pathSeq + "/oxts/timestamps.txt";
    fTimes.open(strPathTimeFile.c_str());
	vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string str;
        getline(fTimes,str);
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

	string strPrefix = pathSeq + "/image_00/data/"; // left
    vstrImages.resize(nTimes);
    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(10) << i;
        vstrImages[i] = strPrefix + ss.str() + ".png";
    }
}

void LoadIMU(const string &pathSeq, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{

	ifstream fTimes;
    string strPathTimeFile = pathSeq + "/oxts/timestamps.txt";
	fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string str;
        getline(fTimes,str);
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

    vAcc.reserve(50000);
    vGyro.reserve(50000);
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

			//if (i<10)
			//{
			//	cout << endl;
			//	cout << data[11] << " " << data[12] << " " << data[13] << " " << data[17] << " " << data[18] << " " << data[19] << endl;
			//	cout << cv::Point3d(data[11],data[12],data[13]) <<  " " << cv::Point3d(data[17],data[18],data[19]) << endl;
			//}
        } 
	}
}
