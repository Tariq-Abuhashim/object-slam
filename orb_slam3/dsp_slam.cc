/**
* 
* Tariq Abuhashim.
* Adopted from dsp_slam examples.
* This reads Kitti images data using actual time stamps.
* Odometry data only can be used.
*
* 13-Dec-2022
*
*/

#include<iostream>
#include<algorithm>
#include<iomanip>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, const float &fps, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        cerr << endl << "Usage: ./dsp_slam path_to_vocabulary path_to_settings path_to_sequence path_to_save_map" << endl;
		cerr << endl << "Examples:" << endl;
		cerr << endl << "./dsp_slam Vocabulary/ORBvoc.bin /media/mrt/Whale/data/kitti/07/KITTI04-12.yaml /media/mrt/Whale/data/kitti/07/" << endl;
        return 1;
    }

    cv::FileStorage fSettings(string(argv[2]), cv::FileStorage::READ);
    float fps = fSettings["Camera.fps"];

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    cout << "Loading images ..." << endl;
    LoadImages(string(argv[3]), fps, vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    cout << "Creating SLAM object ..." << endl;
    ORB_SLAM3::System SLAM(argv[1],argv[2], ORB_SLAM3::System::STEREO, true, 0, argv[3]);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

	/* System test code */

	if (true) {
		std::cout << "\n  Runnig a TEST ONLY loop ... \n" << std::endl;
		for(int ni=0; ni<nImages; ni++)
		{
			cout << "Loading image at: " << string(vstrImageLeft[ni]) << endl;
			usleep(1);
		}
		cv::waitKey(0);
		std::cout << "\n  Shutting down object slam ... \n" << std::endl;
		SLAM.Shutdown();

		std::cout << "\n  Completed ... \n" << std::endl;
		return 0;
	}

    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the images to the SLAM system
		//cout << vstrImageLeft[ni] << endl;
		//cout << vstrImageRight[ni] << endl;
		//cout << vTimestamps[ni] << endl;
        SLAM.TrackStereo(imLeft, imRight, tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T = 0.0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T- ttrack)*1e6)));

		if(argc == 5)
			SLAM.SaveMapCurrentFrame(string(argv[4]), ni);

    }

	if(argc == 5)
    	SLAM.SaveEntireMap(string(argv[4]));

    cv::waitKey(0);
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

    return 0;
}

void LoadImages(const string &strPathToSequence, const float &fps, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    //string strPathTimeFile = strPathToSequence + "/2011_09_30_drive_0018_sync/times.txt"; // kitti raw data
	string strPathTimeFile = strPathToSequence + "/times.txt"; // kitti 07 odometry
    fTimes.open(strPathTimeFile.c_str());
    float dt = 1. / fps;
    float t = 0.;
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            vTimestamps.push_back(t);
            t += dt;
        }
    }

    //string strPrefixLeft = strPathToSequence + "/2011_09_30_drive_0018_sync/image_00/data/"; // kitti raw data
    //string strPrefixRight = strPathToSequence + "/2011_09_30_drive_0018_sync/image_01/data/"; // kitti raw data
	string strPrefixLeft = strPathToSequence + "/image_0/"; // kitti 07 odometry
    string strPrefixRight = strPathToSequence + "/image_1/"; // kitti 07 odometry
    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);


    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i; // 6 for odometry data, 10 for raw data
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
