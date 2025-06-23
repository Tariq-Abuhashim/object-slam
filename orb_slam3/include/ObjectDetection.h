/**
* Tariq updated - Oct, 2022
* Tariq updated - Jan, 2023
**/

#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

# include <mutex>
# include <Eigen/Dense>
# include <vector>

namespace ORB_SLAM3
{

class ObjectDetection {
public:
    // Detction from LiDAR frame, with initial pose, surface LiDAR points, rays and depth measurement
    ObjectDetection(const Eigen::Matrix4f &T, const Eigen::MatrixXf &Pts, const Eigen::MatrixXf &Rays,
                    const Eigen::VectorXf &Depth, const Eigen::Vector3f &Box);

    ObjectDetection();  // Detection from Mono frame
    void SetPoseMeasurementSim3(const Eigen::Matrix4f &T);
    void SetPoseMeasurementSE3(const Eigen::Matrix4f &T);
    std::vector<int> GetFeaturePoints();
    void AddFeaturePoint(const int &i);
    int NumberOfPoints();

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix4f Sim3Tco;
    Eigen::Matrix4f SE3Tco;
    float scale;
    Eigen::Matrix3f Rco;
    Eigen::Vector3f tco;
    Eigen::MatrixXf SurfacePoints;
    Eigen::MatrixXf RayDirections;
    Eigen::VectorXf DepthObs;
    Eigen::MatrixXf background_rays;
    std::vector<int> mvKeysIndices;
    int nRays;
    int nPts;
    bool isNew;
    bool isGood;
    std::mutex mMutexFeatures;
    std::mutex mMutexDetection;

	Eigen::Vector3f Box;
};
}


#endif //OBJECTDETECTION_H
