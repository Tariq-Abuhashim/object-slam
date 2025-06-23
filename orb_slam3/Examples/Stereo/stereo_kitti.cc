/**
* 
* Tariq Abuhashim
* Adopted from ORBSLAM3 examples
* Kitti Stereo example
* This reads Kitti images using actual time stamps
* Raw data can be used (previous examples only use odometry data)
* KITTI.yaml has been updated to capture data and Inertial changes
*
* 21-Oct-2022
*
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &pathSeq, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[3])+"/2011_09_26_drive_0022_sync", vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);  

    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

//#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//#else
//        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
//#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);

//#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//#else
//        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
//#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}

void LoadImages(const string &pathSeq, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    //string strPathTimeFile = pathSeq + "/times.txt";
	string strPathTimeFile = pathSeq + "/oxts/timestamps.txt";
    fTimes.open(strPathTimeFile.c_str());
	vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
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

    string strPrefixLeft = pathSeq + "/image_00/data/";
    string strPrefixRight = pathSeq + "/image_01/data/";
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);
    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(10) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
