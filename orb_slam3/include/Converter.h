/**
* Tariq updated - Oct, 2022
* Tariq updated - Mar, 2023 - major update to orbslam3 compatability
**/

#ifndef CONVERTER_H
#define CONVERTER_H

#include <pangolin/pangolin.h> // Object-SLAM
#include <opencv2/core/core.hpp>

#include <Eigen/Dense>
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include "Thirdparty/Sophus/sophus/geometry.hpp"
#include "Thirdparty/Sophus/sophus/sim3.hpp"

namespace ORB_SLAM3
{

class Converter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

	//static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);
    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);			// cv::Mat		-> g2o::SE3Quat
    static g2o::SE3Quat toSE3Quat(const Sophus::SE3f &T);       // Sophus::SE3f -> g2o::SE3Quat 
	static g2o::SE3Quat toSE3Quat(const Eigen::Matrix4f &T);	// Matrix4f 	-> g2o::SE3Quat
	static g2o::Sim3 	toSim3(const Eigen::Matrix4f &T); 		// Matrix4f 	-> g2o::Sim3

	//TODO: Sophus migration, to be deleted in the future
    static Sophus::SE3<float> toSophus(const cv::Mat& T);		// cv::Mat		-> Sophus::SE3<float>
    static Sophus::Sim3f toSophus(const g2o::Sim3& S);			// g2o::Sim3	-> Sophus::Sim3f

	static cv::Mat toCvMat(const g2o::SE3Quat &SE3);			// g2o::SE3Quat		 -> cv::Mat
	static cv::Mat toCvMat(const g2o::Sim3 &Sim3);				// g2o::Sim3    	 -> cv::Mat
    static cv::Mat toCvMat(const Eigen::Matrix<float,3,1> &m);  // Vector3f			 -> cv::Mat
    static cv::Mat toCvMat(const Eigen::Matrix<float,3,3> &m);  // Matrix3f			 -> cv::Mat
    static cv::Mat toCvMat(const Eigen::Matrix<float,3,4> &m);  // Matrix<float,3,4> -> cv::Mat
    static cv::Mat toCvMat(const Eigen::Matrix<float,4,4> &m);  // Matrix4f 		 -> cv::Mat
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m); // Vector3d			 -> cv::Mat
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,3> &m); // Matrix3d 		 -> cv::Mat
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m); // Matrix4d 		 -> cv::Mat
    static cv::Mat toCvMat(const Eigen::MatrixXf &m); 			// MatrixXf 		 -> cv::Mat
    static cv::Mat toCvMat(const Eigen::MatrixXd &m); 			// MatrixXd 		 -> cv::Mat

	static Eigen::Matrix<float,3,1> 	toVector3f(const cv::Mat &cvVector);    // cv::Mat		-> Vector3f
    static Eigen::Matrix<double,3,1> 	toVector3d(const cv::Mat &cvVector);    // cv::Mat		-> Vector3d
	static Eigen::Matrix<float,3,3> 	toMatrix3f(const cv::Mat &cvMat3);      // cv::Mat		-> Matrix3f
	static Eigen::Matrix<float,4,4> 	toMatrix4f(const cv::Mat &cvMat4);      // cv::Mat		-> Matrix4f
    static Eigen::Matrix<double,3,3> 	toMatrix3d(const cv::Mat &cvMat3);      // cv::Mat		-> Matrix3d
    static Eigen::Matrix<double,4,4> 	toMatrix4d(const cv::Mat &cvMat4);      // cv::Mat		-> Matrix4d
	static Eigen::Matrix<double,3,1> 	toVector3d(const cv::Point3f &cvPoint); // Point3f		-> Vector3d
	static Eigen::Matrix<float,4,4>   	toMatrix4f(const g2o::SE3Quat &SE3);	// g2o::SE3Quat	-> Matrix4f

    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);
    static cv::Mat tocvSkewMatrix(const cv::Mat &v);
    static bool isRotationMatrix(const cv::Mat &R);
    static std::vector<float> toEuler(const cv::Mat &R);
	static std::vector<float> toQuaternion(const cv::Mat &M);

    static pangolin::OpenGlMatrix toMatrixPango(const Eigen::Matrix4f &T);

};

}// namespace ORB_SLAM

#endif // CONVERTER_H
