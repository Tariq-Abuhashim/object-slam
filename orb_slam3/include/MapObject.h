
/**
* Tariq updated - Jun, 2022
**/

#ifndef MAPOBJECT_H
#define MAPOBJECT_H


#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>

// COVINS
#include "comm/communicator.hpp"

namespace ORB_SLAM3 {

class KeyFrame;
class Map;
class Frame;

class MapObject {
public:
    MapObject(const Eigen::Matrix4f &T, const Eigen::VectorXf &vCode, const Eigen::Vector3f &inBox, KeyFrame *pRefKF, Map *pMap); // covins: changes Eigen::Vector<float, 64> to Eigen::VectorXf &vCode
    MapObject(KeyFrame *pRefKF, Map *pMap);

    void AddObservation(KeyFrame *pKF, int idx);
    int Observations();
    std::map<KeyFrame*,size_t> GetObservations();
    void SetObjectPoseSim3(const Eigen::Matrix4f &Two);
    void SetObjectPoseSE3(const Eigen::Matrix4f &Two);
    void SetShapeCode(const Eigen::VectorXf &code); // covins: changes Eigen::Vector<float, 64> to Eigen::VectorXf &vCode
    void UpdateReconstruction(const Eigen::Matrix4f &T, const Eigen::VectorXf &vCode); // covins: changes Eigen::Vector<float, 64> to Eigen::VectorXf &vCode
    Eigen::Matrix4f GetPoseSim3();
    Eigen::Matrix4f GetPoseSE3();
    Eigen::VectorXf GetShapeCode(); // covins: changes Eigen::Vector<float, 64> to Eigen::VectorXf &vCode
    int GetIndexInKeyFrame(KeyFrame *pKF);
    void EraseObservation(KeyFrame *pKF);
    void SetBadFlag();
    bool isBad();
    void SetVelocity(const Eigen::Vector3f &v);
    void Replace(MapObject *pMO);
    bool IsInKeyFrame(KeyFrame *pKF);
    KeyFrame* GetReferenceKeyFrame();

    std::vector<MapPoint*> GetMapPointsOnObject();
    void AddMapPoints(MapPoint *pMP);
    void RemoveOutliersSimple();
    void RemoveOutliersModel();
    void ComputeCuboidPCA(bool updatePose);
    void EraseMapPoint(MapPoint *pMP);

    void SetRenderId(int id);
    int GetRenderId();
    void SetDynamicFlag();
    bool isDynamic();

    #ifdef COVINS_MOD
    bool sent_once_ = false;
    virtual auto ConvertToMsg(covins::MsgObject &msg, KeyFrame* kf_ref, bool is_update, size_t cliend_id)->void;
    #endif

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix4f SE3Two;
    Eigen::Matrix4f SE3Tow;
    Eigen::Matrix4f Sim3Two;
    Eigen::Matrix4f Sim3Tow;
    Eigen::Matrix3f Rwo;
    Eigen::Vector3f two;
    float scale;
    float invScale;
    Eigen::VectorXf vShapeCode;  // covins: changes Eigen::Vector<float, 64> to Eigen::VectorXf &vCode

    // Keyframes observing the point and associated index in keyframe
    std::map<KeyFrame *, size_t> mObservations;

    // Reference KeyFrame
    KeyFrame *mpRefKF;
    KeyFrame *mpNewestKF;
    long unsigned int mnBALocalForKF;
    long unsigned int mnAssoRefID;
    long unsigned int mnFirstKFid;

    // variables used for loop closing
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    long unsigned int mnLoopObjectForKF;
    long unsigned int mnBAGlobalForKF;
    MapObject *mpReplaced;
    Eigen::Matrix4f mTwoGBA;

    bool reconstructed;
    std::set<MapPoint*> map_points;

    // cuboid
    float w;
    float h;
    float l;

    // Bad flag (we do not currently erase MapObject from memory)
    bool mbBad;
    bool mbDynamic;
    Eigen::Vector3f velocity;
    Map *mpMap;

    int nObs;
    static int nNextId;
    int mnId; // Object ID
    int mRenderId; // Object ID in the renderer
    Eigen::MatrixXf vertices;
    Eigen::MatrixXi faces;

    std::mutex mMutexObject;
    std::mutex mMutexFeatures;

    static bool lId(MapObject* pMO1, MapObject* pMO2){
        return pMO1->mnId < pMO2->mnId;
    }

};

}
#endif //MAPOBJECT_H
