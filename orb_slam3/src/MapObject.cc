
/**
* Tariq updated - Jun, 2022
**/

#include "MapObject.h"
#include "Converter.h"
#include <eigen3/Eigen/Dense>

// COVINS
#include <covins/covins_base/utils_base.hpp>
#include <covins/covins_base/typedefs_base.hpp>

namespace ORB_SLAM3
{

int MapObject::nNextId = 0;

#ifdef COVINS_MOD
auto MapObject::ConvertToMsg(covins::MsgObject &msg, KeyFrame *kf_ref, bool is_update, size_t cliend_id)->void {
    std::unique_lock<std::mutex> loc_obs(mMutexFeatures);
    std::unique_lock<std::mutex> lock_pos(mMutexObject);

    msg.is_update_msg = is_update;

    msg.id.first = mnId;
    msg.id.second = cliend_id;

    if(!kf_ref) {
        std::cout << COUTERROR << "MO " << mnId << ": no kf_ref given" << std::endl;
        exit(-1);
    }

    msg.id_reference.first = kf_ref->mnId;
    msg.id_reference.second = cliend_id;

	msg.w = w;
	msg.h = h;
	msg.l = l;
	msg.vShapeCode = vShapeCode;

    //covins::TypeDefs::TransformType T_w_sref = covins::Utils::ToEigenMat44d(kf_ref->GetImuPose());
    //covins::TypeDefs::TransformType T_sref_w = T_w_sref.inverse();
    //covins::TypeDefs::Matrix3Type R_sref_w = T_sref_w.block<3,3>(0,0);
    //covins::TypeDefs::Vector3Type t_sref_w = T_sref_w.block<3,1>(0,3);
    //covins::TypeDefs::Vector3Type pos_w = covins::Utils::ToEigenVec3d(mWorldPos);
    //msg.pos_ref = R_sref_w * pos_w + t_sref_w;

/*
    if(!is_update) {
        for(std::map<KeyFrame*,std::tuple<int,int>>::const_iterator mit=mObservations.begin();mit!=mObservations.end();++mit){
            if(mit->first != nullptr){
                msg.observations.insert(std::make_pair(make_pair(mit->first->mnId,cliend_id),mit->second));
            }
        }
    }
*/
}
#endif

MapObject::MapObject(const Eigen::Matrix4f &T, const Eigen::VectorXf &vCode, const Eigen::Vector3f &inBox, KeyFrame *pRefKF, Map *pMap) :  // Tariq - Vector<float, 64> to VectorXf
        mpRefKF(pRefKF), mpNewestKF(pRefKF), mnBALocalForKF(0), mnAssoRefID(0), mnFirstKFid(pRefKF->mnId),
        mnCorrectedByKF(0), mnCorrectedReference(0), mnLoopObjectForKF(0), mnBAGlobalForKF(0),
        w(inBox.coeff(0)), l(inBox.coeff(1)), h(inBox.coeff(2)), mbBad(false), mbDynamic(false), mpMap(pMap), nObs(0), mRenderId(-1)
{
    // Transformation Matrix in Sim3
    Sim3Two = T;
    Sim3Tow = Sim3Two.inverse();

    // Decompose T into Rotation, translation and scale
    Rwo = T.topLeftCorner<3, 3>();
    // scale is fixed once the object is initialized
    scale = pow(Rwo.determinant(), 1./3.);
    invScale = 1. / scale;
    Rwo /= scale;
    two = T.topRightCorner<3, 1>();

    // Transformation Matrix in SE3
    SE3Two = Eigen::Matrix4f::Identity();
    SE3Two.topLeftCorner<3, 3>() = Rwo;
    SE3Two.topRightCorner<3, 1>() = two;
    SE3Tow = SE3Two.inverse();

    vShapeCode = vCode;
    velocity = Eigen::Vector3f::Zero();
    mnId = nNextId++;
}

MapObject::MapObject(KeyFrame *pRefKF, Map *pMap) :
        mpRefKF(pRefKF), mpNewestKF(pRefKF), mnBALocalForKF(0), mnAssoRefID(0), mnFirstKFid(pRefKF->mnId),
        mnCorrectedByKF(0), mnCorrectedReference(0), mnLoopObjectForKF(0), mnBAGlobalForKF(0),
        reconstructed(false), w(1.), h(1.), l(1.), mbBad(false), mbDynamic(false), mpMap(pMap), nObs(0), mRenderId(-1)
{
    mnId = nNextId++;
    scale = 1.;
    invScale = 1.;
    vShapeCode = Eigen::VectorXf::Zero(64); // Eigen::Vector<float, 64>::Zero() to Eigen::VectorXf::Zero(64)
}

// add a keyframe
void MapObject::AddObservation(KeyFrame *pKF, int idx)
{
    unique_lock<mutex> lock(mMutexObject);
    if(!mObservations.count(pKF))
        nObs++;
    mObservations[pKF]=idx;
    mpNewestKF = pKF;
}

// return keyframe observations
std::map<KeyFrame*, size_t> MapObject::GetObservations()
{
    unique_lock<mutex> lock(mMutexObject);
    return mObservations;
}

// return number of observations (ie keyframes)
void MapObject::EraseObservation(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexObject);
    if(mObservations.count(pKF))
    {
        nObs--;
        mObservations.erase(pKF);

        if(mpRefKF==pKF) // if the erased keyframe is the reference, update mpRefKF
            mpRefKF=mObservations.begin()->first;

        if(mpNewestKF == pKF)  // if the erased keyframe is the newest, update mpNewestKF
        {
            int mnLargestKFId = 0;
            KeyFrame *pNewestKF = static_cast<KeyFrame*>(nullptr);
            for(std::map<KeyFrame *, size_t>::iterator mit = mObservations.begin(), mend = mObservations.end(); mit != mend; mit++)
            {
                KeyFrame* plKF = mit->first;
                if (plKF->mnId > mnLargestKFId)
                {
                    mnLargestKFId = plKF->mnId;
                    pNewestKF = plKF;
                }
            }
            mpNewestKF = pNewestKF;
        }

    }
}

// return number of observations (ie keyframes)
int MapObject::Observations()
{
    unique_lock<mutex> lock(mMutexObject);
    return nObs;
}

// label the object bad and delete all its observations
void MapObject::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock(mMutexObject);
        mbBad = true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapObjectMatch(mit->second);
    }
}

// return the state of the object
bool MapObject::isBad()
{
    unique_lock<mutex> lock(mMutexObject);
    return mbBad;
}

// return the index of the object observation in a keyframe
int MapObject::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexObject);
    if (mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

// check if the object is visible in this keyframe
bool MapObject::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexObject);
    return (mObservations.count(pKF));
}

// replace by a new object
void MapObject::Replace(MapObject *pMO)
{
    if(pMO->mnId==this->mnId) // if its the same object, skip
        return;

    map<KeyFrame*,size_t> obs; // set old object bad, delete observations and copy object to mpReplaced
    {
        unique_lock<mutex> lock1(mMutexObject);
        obs = mObservations;
        mObservations.clear();
        mbBad = true;
        mpReplaced = pMO;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMO->IsInKeyFrame(pKF)) // if the new object is in the keyframe observation as the old object
        {
            pKF->ReplaceMapObjectMatch(mit->second, pMO); // replace the object in the keyframe
            pMO->AddObservation(pKF, mit->second); // add the keyframe to new object observations
        }
        else // if the new object is NOT in the keyframe observation as the old object
        {
            pKF->EraseMapObjectMatch(mit->second); // erase the object in the keyframe
        }
    }

    this->SetBadFlag();
}

// return the reference keyframe
KeyFrame* MapObject::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexObject);
    return mpRefKF;
}

// set the object pose
void MapObject::SetObjectPoseSim3(const Eigen::Matrix4f &Two)
{
    unique_lock<mutex> lock(mMutexObject);
    Sim3Two = Two;
    Sim3Tow = Sim3Two.inverse();

    // Decompose T into Rotation, translation and scale
    Rwo = Two.topLeftCorner<3, 3>();
    // scale is fixed once the object is initialized
    scale = pow(Rwo.determinant(), 1./3.);
    invScale = 1. / scale;
    Rwo /= scale;
    two = Two.topRightCorner<3, 1>();

    // Transformation Matrix in SE3
    SE3Two = Eigen::Matrix4f::Identity();
    SE3Two.topLeftCorner<3, 3>() = Rwo;
    SE3Two.topRightCorner<3, 1>() = two;
    SE3Tow = SE3Two.inverse();
}

// set the object pose
void MapObject::SetObjectPoseSE3(const Eigen::Matrix4f &Two)
{
    unique_lock<mutex> lock(mMutexObject);
    SE3Two = Two;
    SE3Tow = SE3Two.inverse();
    Rwo = SE3Two.topLeftCorner<3, 3>();
    two = SE3Two.topRightCorner<3, 1>();
    Sim3Two.topLeftCorner<3, 3>() = Rwo * scale;
    Sim3Two.topRightCorner<3, 1>() = two;
    Sim3Tow = Sim3Two.inverse();
}

// set the object shape codes for deepSDF
void MapObject::SetShapeCode(const Eigen::VectorXf &code)  // Vector<float, 64> to VectorXf
{
    unique_lock<mutex> lock(mMutexObject);
    vShapeCode = code;
}

void MapObject::UpdateReconstruction(const Eigen::Matrix4f &T, const Eigen::VectorXf &vCode)  // Vector<float, 64> to VectorXf
{
    SetObjectPoseSim3(T);
    SetShapeCode(vCode);
}

std::vector<MapPoint*> MapObject::GetMapPointsOnObject()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return vector<MapPoint *>(map_points.begin(), map_points.end());
}

void MapObject::RemoveOutliersSimple()
{
    // First pass, remove all the outliers marked by ORB-SLAM
    int n_pts = 0;
    Eigen::Vector3f x3D_mean = Eigen::Vector3f::Zero();
    for (auto pMP : GetMapPointsOnObject())
    {
        if (!pMP)
            continue;
        if (pMP->isBad())
            this->EraseMapPoint(pMP);
        else
        {
            x3D_mean += Converter::toVector3f(pMP->GetWorldPos());
            n_pts++;
        }
    }

    if (n_pts == 0)
    {
        this->SetBadFlag();
        return;
    }

    // Second pass, remove obvious outliers
    x3D_mean /= n_pts;
    for (auto pMP : GetMapPointsOnObject())
    {
        Eigen::Vector3f x3Dw = Converter::toVector3f(pMP->GetWorldPos());
        if ((x3Dw - x3D_mean).norm() > 1.0)
        {
            this->EraseMapPoint(pMP);
        }
    }
}

void MapObject::RemoveOutliersModel()
{
    // sanity check: too few number of vertices
    if (vertices.rows() <= 10)
        return;

    float xmin = vertices.col(0).minCoeff();
    float xmax = vertices.col(0).maxCoeff();
    float ymin = vertices.col(1).minCoeff();
    float ymax = vertices.col(1).maxCoeff();
    float zmin = vertices.col(2).minCoeff();
    float zmax = vertices.col(2).maxCoeff();

    w = (xmax - xmin) * scale;
    h = (ymax - ymin) * scale;
    l = (zmax - zmin) * scale;
    float sx = 1.2;
    float sy = 1.5;
    float sz = 1.2;

    auto mvpMapPoints = GetMapPointsOnObject();
    for (auto pMP : mvpMapPoints)
    {
        if (!pMP)
            continue;

        if (pMP->isBad())
        {
            this->EraseMapPoint(pMP);
        }
        else
        {
            auto x3Dw = Converter::toVector3f(pMP->GetWorldPos());
            auto x3Do = invScale * Rwo.inverse() * x3Dw - invScale * Rwo.inverse() * two;
            if (x3Do(0) > sx * xmax || x3Do(0) < sx * xmin ||
                x3Do(1) > sy * ymax || x3Do(1) < sy * ymin ||
                x3Do(2) > sz * zmax || x3Do(2) < sz * zmin)
            {
                pMP->SetOutlierFlag();
            }
        }
    }
}

void MapObject::ComputeCuboidPCA(bool updatePose)
{
    RemoveOutliersSimple();
    auto mvpMapPoints = GetMapPointsOnObject();
    int N = mvpMapPoints.size();

    if (N == 0)
    {
        this->SetBadFlag();
        return;
    }

    Eigen::Vector3f x3D_mean = Eigen::Vector3f::Zero();
    Eigen::MatrixXf Xpts = Eigen::MatrixXf::Zero(N, 3);
    Eigen::MatrixXf Xpts_shifted = Eigen::MatrixXf::Zero(N, 3);
    for (int i = 0; i < N; i++)
    {
        auto pMP = mvpMapPoints[i];
        cv::Mat x3Dw = pMP->GetWorldPos();
        Xpts(i, 0) = x3Dw.at<float>(0);
        Xpts(i, 1) = x3Dw.at<float>(1);
        Xpts(i, 2) = x3Dw.at<float>(2);
        x3D_mean += Converter::toVector3f(pMP->GetWorldPos());
    }

    x3D_mean /= N;
    for (int i = 0; i < N; i++)
    {
        Xpts_shifted.row(i) = Xpts.row(i) - x3D_mean.transpose();
    }

    auto covX = Xpts_shifted.transpose() * Xpts_shifted;
    // cout << covX << endl;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covX);

    auto eigenvectors = eigensolver.eigenvectors();

    // Get rotation matrix, following ShapeNet definition
    // x : right, y: up, z: back
    Eigen::Matrix3f R;
    // Assume the order of principal axis: y, x, -z
    R.col(0) = eigenvectors.col(1);
    R.col(1) = eigenvectors.col(0);
    R.col(2) = -eigenvectors.col(2);

    // Check if det(R) = -1
    if (R.determinant() < 0)
        R.col(0) = -R.col(0);

    // Check if y direction is pointing upward by comparing its angle between camera
    auto neg_y = Eigen::Vector3f(0.f, -1.f, 0.f);
    if (neg_y.dot(R.col(1)) < 0)
    {
        R.col(0) = -R.col(0);
        R.col(1) = -R.col(1);
    }

    int lo = int (0.05 * N);  // percentile threshold
    int hi = int (0.95 * N);
    auto Xpts_o = R.inverse() * Xpts.transpose(); // 3 x N
    Eigen::VectorXf x, y, z;
    x = Xpts_o.row(0);  // x corresponds to w
    y = Xpts_o.row(1);  // y corresponds to h
    z = Xpts_o.row(2);  // z corresponds to l
    // Sort the vectors
    std::sort(x.data(),x.data() + x.size());
    std::sort(y.data(),y.data() + y.size());
    std::sort(z.data(),z.data() + z.size());

    // PCA box dims
    w = (x(hi) - x(lo));
    h = (y(hi) - y(lo));
    l = (z(hi) - z(lo));
    Eigen::Vector3f cuboid_centre_o((x(hi) + x(lo)) / 2., (y(hi) + y(lo)) / 2., (z(hi) + z(lo)) / 2.);
    Eigen::Vector3f cuboid_centre_w = R * cuboid_centre_o;

    // Remove outliers using computed PCA box
    int num_outliers = 0;
    float s = 1.2;
    for (auto pMP : mvpMapPoints)
    {
        if (!pMP)
            continue;

        if (pMP->isBad())
        {
            this->EraseMapPoint(pMP);
        }
        else
        {
            auto x3Dw = Converter::toVector3f(pMP->GetWorldPos());
            auto x3Do = R.inverse() * x3Dw - R.inverse() * cuboid_centre_w;
            if (x3Do(0) > s * w / 2 || x3Do(0) < -s * w / 2 ||
                x3Do(1) > s * h / 2 || x3Do(1) < -s * h / 2 ||
                x3Do(2) > s * l / 2 || x3Do(2) < -s * l / 2)
            {
                pMP->SetOutlierFlag();
                num_outliers++;
            }
        }
    }
    // Update object pose with pose computed by PCA, only for the very first few frames
    if (updatePose)
    {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.topLeftCorner(3, 3) = 0.40 * l * R;
        // cout << R.determinant() << " " << endl;
        // cout << pow(T.topLeftCorner(3, 3).determinant(), 1./3) << endl;
        T.topRightCorner(3, 1) = cuboid_centre_w;
        SetObjectPoseSim3(T);
    }
}

// add 3D MapPoints that are part of the object surface
void MapObject::AddMapPoints(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexFeatures);
    map_points.insert(pMP);
}

// remove 3D MapPoints that are not part of the object surface and set them as being bad
void MapObject::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexFeatures);
    map_points.erase(pMP);
    pMP->SetBadFlag();
}

// set the object velocity
void MapObject::SetVelocity(const Eigen::Vector3f &v)
{
    unique_lock<mutex> lock(mMutexObject);
    velocity = v;
}

// return object pose in Sim3 format
Eigen::Matrix4f MapObject::GetPoseSim3()
{
    unique_lock<mutex> lock(mMutexObject);
    return Sim3Two;
}

// return object pose in SE3 format
Eigen::Matrix4f MapObject::GetPoseSE3()
{
    unique_lock<mutex> lock(mMutexObject);
    return SE3Two;
}

// return the object shape codes
Eigen::VectorXf MapObject::GetShapeCode() // Vector<float, 64> to VectorXf
{
    unique_lock<mutex> lock(mMutexObject);
    return vShapeCode;
}

int MapObject::GetRenderId()
{
    unique_lock<mutex> lock(mMutexObject);
    return mRenderId;
}

void MapObject::SetRenderId(int id)
{
    unique_lock<mutex> lock(mMutexObject);
    mRenderId = id;
}

// set the object dynamic
void MapObject::SetDynamicFlag()
{
    unique_lock<mutex> lock(mMutexObject);
    mbDynamic = true;
}

// check if object is set dynamic
bool MapObject::isDynamic()
{
    unique_lock<mutex> lock(mMutexObject);
    return mbDynamic;
}

}

