
/**
* Tariq updated - Oct, 2022
* Tariq updated - Mar, 2023
**/

#include "Converter.h"

namespace ORB_SLAM3
{

/* checked */
std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}

/* 	checked 
	cv::Mat -> g2o::SE3Quat
*/
g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}

/* 	checked 
	Sophus::SE3f -> g2o::SE3Quat
*/
g2o::SE3Quat Converter::toSE3Quat(const Sophus::SE3f &T)
{
    return g2o::SE3Quat(T.unit_quaternion().cast<double>(), T.translation().cast<double>());
}

/* 	checked 
	Matrix4f -> g2o::SE3Quat
*/
g2o::SE3Quat Converter::toSE3Quat(const Eigen::Matrix4f &T)
{
    Eigen::Matrix<double, 3, 3> R = T.topLeftCorner<3, 3>().cast<double>();
    Eigen::Matrix<double, 3, 1> t = T.topRightCorner<3, 1>().cast<double>();

    return g2o::SE3Quat(R, t);
}

/* checked 
	Matrix4f -> g2o::Sim3
*/
g2o::Sim3 Converter::toSim3(const Eigen::Matrix4f &T)
{
    Eigen::Matrix3d sR = T.topLeftCorner<3, 3>().cast<double>();
    double s = pow(sR.determinant(), 1. / 3);
    Eigen::Matrix3d R = sR / s;
    Eigen::Vector3d t = T.topRightCorner<3, 1>().cast<double>();

    return g2o::Sim3(R, t, s);
}

/* 	checked 
	v::Mat -> Sophus::SE3<float>
*/
Sophus::SE3<float> Converter::toSophus(const cv::Mat &T) {
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(T.rowRange(0,3).colRange(0,3));
    Eigen::Quaternionf q(eigMat.cast<float>());

    Eigen::Matrix<float,3,1> t = toVector3d(T.rowRange(0,3).col(3)).cast<float>();

    return Sophus::SE3<float>(q,t);
}

/* 	checked 
	g2o::Sim3 -> Sophus::Sim3f
*/
Sophus::Sim3f Converter::toSophus(const g2o::Sim3& S) {
    return Sophus::Sim3f(Sophus::RxSO3d((float)S.scale(), S.rotation().matrix()).cast<float>() ,
                         S.translation().cast<float>());
}

/* 	checked 
	g2o::SE3Quat -> cv::Mat
*/
cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();

    return toCvMat(eigMat);
}

/* 	checked 
	g2o::Sim3 -> cv::Mat
*/
cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
{
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
    Eigen::Vector3d eigt = Sim3.translation();
    double s = Sim3.scale();

    return toCvSE3(s*eigR,eigt);
}

/* 	checked 
	Vector3f -> cv::Mat
*/
cv::Mat Converter::toCvMat(const Eigen::Vector3f &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
        cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

/* 	checked 
	Matrix3f -> cv::Mat
*/
cv::Mat Converter::toCvMat(const Eigen::Matrix3f &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

/* 	checked 
	Matrix<float,3,4> -> cv::Mat
*/
cv::Mat Converter::toCvMat(const Eigen::Matrix<float,3,4> &m)
{
    cv::Mat cvMat(3,4,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

/* 	checked 
	Matrix4f -> cv::Mat
*/
cv::Mat Converter::toCvMat(const Eigen::Matrix4f &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

/* 	checked 
	Vector3d -> cv::Mat
*/
cv::Mat Converter::toCvMat(const Eigen::Vector3d &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

/* 	checked 
	Matrix3d -> cv::Mat
*/
cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

/* 	checked 
	Matrix4d -> cv::Mat
*/
cv::Mat Converter::toCvMat(const Eigen::Matrix4d &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

/* 	checked 
	MatrixXf -> cv::Mat
*/
cv::Mat Converter::toCvMat(const Eigen::MatrixXf &m)
{
    cv::Mat cvMat(m.rows(),m.cols(),CV_32F);
    for(int i=0;i<m.rows();i++)
        for(int j=0; j<m.cols(); j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

/*	checked 
	MatrixXd -> cv::Mat 
*/
cv::Mat Converter::toCvMat(const Eigen::MatrixXd &m)
{
    cv::Mat cvMat(m.rows(),m.cols(),CV_32F);
    for(int i=0;i<m.rows();i++)
        for(int j=0; j<m.cols(); j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

/* checked */
Eigen::Matrix<float,3,1> Converter::toVector3f(const cv::Mat &cvVector)
{
    Eigen::Matrix<float,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

/* checked */
Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

/* checked */
Eigen::Matrix<float,3,3> Converter::toMatrix3f(const cv::Mat &cvMat3)
{
    Eigen::Matrix<float, 3, 3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
            cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
            cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

/* checked */
Eigen::Matrix<float,4,4> Converter::toMatrix4f(const cv::Mat &cvMat4)
{
    Eigen::Matrix4f M;
    M << cvMat4.at<float>(0,0), cvMat4.at<float>(0,1), cvMat4.at<float>(0,2), cvMat4.at<float>(0, 3),
         cvMat4.at<float>(1,0), cvMat4.at<float>(1,1), cvMat4.at<float>(1,2), cvMat4.at<float>(1, 3),
         cvMat4.at<float>(2,0), cvMat4.at<float>(2,1), cvMat4.at<float>(2,2), cvMat4.at<float>(2, 3),
         cvMat4.at<float>(3,0), cvMat4.at<float>(3,1), cvMat4.at<float>(3,2), cvMat4.at<float>(3, 3);

    return M;
}

/* checked */
Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

/* checked */
Eigen::Matrix<double,4,4> Converter::toMatrix4d(const cv::Mat &cvMat4)
{
    Eigen::Matrix<double,4,4> M;

    M << cvMat4.at<float>(0,0), cvMat4.at<float>(0,1), cvMat4.at<float>(0,2), cvMat4.at<float>(0,3),
         cvMat4.at<float>(1,0), cvMat4.at<float>(1,1), cvMat4.at<float>(1,2), cvMat4.at<float>(1,3),
         cvMat4.at<float>(2,0), cvMat4.at<float>(2,1), cvMat4.at<float>(2,2), cvMat4.at<float>(2,3),
         cvMat4.at<float>(3,0), cvMat4.at<float>(3,1), cvMat4.at<float>(3,2), cvMat4.at<float>(3,3);

    return M;
}

/* checked */
Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

/* checked */
cv::Mat Converter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    return cvMat.clone();
}

/* checked */
cv::Mat Converter::tocvSkewMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

/* checked */
bool Converter::isRotationMatrix(const cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  cv::norm(I, shouldBeIdentity) < 1e-6;

}

/* checked */
std::vector<float> Converter::toEuler(const cv::Mat &R)
{
    assert(isRotationMatrix(R));
    float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );

    bool singular = sy < 1e-6;

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<float>(2,1) , R.at<float>(2,2));
        y = atan2(-R.at<float>(2,0), sy);
        z = atan2(R.at<float>(1,0), R.at<float>(0,0));
    }
    else
    {
        x = atan2(-R.at<float>(1,2), R.at<float>(1,1));
        y = atan2(-R.at<float>(2,0), sy);
        z = 0;
    }

    std::vector<float> v_euler(3);
    v_euler[0] = x;
    v_euler[1] = y;
    v_euler[2] = z;

    return v_euler;
}

/* checked */
std::vector<float> Converter::toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}

/* checked */
pangolin::OpenGlMatrix Converter::toMatrixPango(const Eigen::Matrix4f &T) // DSP - tariq
{
    pangolin::OpenGlMatrix M;
    M.SetIdentity();

    M.m[0] = T(0,0);
    M.m[1] = T(1,0);
    M.m[2] = T(2,0);
    M.m[3]  = 0.0;

    M.m[4] = T(0,1);
    M.m[5] = T(1,1);
    M.m[6] = T(2,1);
    M.m[7]  = 0.0;

    M.m[8] = T(0,2);
    M.m[9] = T(1,2);
    M.m[10] = T(2,2);
    M.m[11]  = 0.0;

    M.m[12] = T(0,3);
    M.m[13] = T(1, 3);
    M.m[14] = T(2, 3);
    M.m[15]  = 1.0;

    return M;
}



Eigen::Matrix4f Converter::toMatrix4f(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix4f eigMat = SE3.to_homogeneous_matrix().cast<float>();
    return eigMat;
}







/*


Eigen::Matrix4f Converter::toMatrix4f(const g2o::Sim3 &Sim3) // DSP - tariq
{
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f eigR = Sim3.rotation().toRotationMatrix().cast<float>();
    Eigen::Vector3f eigt = Sim3.translation().cast<float>();
    float s = Sim3.scale();
    T.topLeftCorner<3, 3>() = s * eigR;
    T.topRightCorner<3, 1>() = eigt;
    return T;
}


std::vector<float> Converter::toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}
*/






} //namespace ORB_SLAM
