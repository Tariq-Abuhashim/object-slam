
/**
* Tariq updated - Aug - 2022
* Tariq updated - Nov - 2022
*/


#include "Optimizer.h"

#include <complex>

#include <Eigen/StdVector>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/factory.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "G2oTypes.h"
#include "Converter.h"

#include <mutex>

#include "OptimizableTypes.h"
#include "ObjectPoseGraph.h" // defines EdgeSE3LieAlgebra and EdgeSE3LieAlgebra_2

namespace ORB_SLAM3
{

G2O_REGISTER_TYPE(VERTEX_SE3:OBJ, VertexSE3Object);
G2O_REGISTER_TYPE(EDGE_SE3:LIE_ALGEBRA, EdgeSE3LieAlgebra);

int Optimizer::nBAdone = 0;

void Optimizer::GlobalJointBundleAdjustemnt(Map *pMap, int nIterations, bool *pbStopFlag, const unsigned long nLoopKF,
                                       const bool bRobust) {
    vector < KeyFrame * > vpKFs = pMap->GetAllKeyFrames();
    vector < MapPoint * > vpMP = pMap->GetAllMapPoints();
    vector < MapObject * > vpMO = pMap->GetAllMapObjects();
    JointBundleAdjustment(vpKFs, vpMP, vpMO, nIterations, pbStopFlag, nLoopKF, bRobust);
}

void Optimizer::JointBundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                      const std::vector<MapObject *> &vpMO,
                                      int nIterations, bool *pbStopFlag, const unsigned long nLoopKF,
                                      const bool bRobust) {
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());
    vector<bool> vbNotIncludedMO;
    vbNotIncludedMO.resize(vpMO.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;
    long unsigned int maxMPid = 0;

    // Set KeyFrame vertices
    for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
            continue;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId == 0);
        optimizer.addVertex(vSE3);
        if (pKF->mnId > maxKFid)
            maxKFid = pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);
    const float invSigmaObject = 1e3;
    const float thHuberObject = sqrt(0.10 * invSigmaObject);
    const float thHuberObjectSquare = pow(thHuberObject, 2);

    // Set MapPoint vertices
    for (size_t i = 0; i < vpMP.size(); i++) {
        MapPoint *pMP = vpMP[i];
        if (pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        if (pMP->mnId > maxMPid)
            maxMPid = pMP->mnId;

        const map<KeyFrame *, tuple<int,int>> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for (map<KeyFrame *, tuple<int,int>>::const_iterator mit = observations.begin(); mit != observations.end(); mit++) {

            KeyFrame *pKF = mit->first;
            if (pKF->isBad() || pKF->mnId > maxKFid)
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[get<0>(mit->second)];

            if (pKF->mvuRight[get<0>(mit->second)] < 0) {
                Eigen::Matrix<double, 2, 1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                if (bRobust) {
                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;

                optimizer.addEdge(e);
            } else {
                Eigen::Matrix<double, 3, 1> obs;
                const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ(); // BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                e->setInformation(Info);

                if (bRobust) {
                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);
            }
        }

        if (nEdges == 0) {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i] = true;
        } else {
            vbNotIncludedMP[i] = false;
        }
    }

    // Set MapObject Vertices
    for (size_t i = 0; i < vpMO.size(); i++) {
        auto pMO = vpMO[i];

        if (!pMO) {
            vbNotIncludedMO[i] = true;
            continue;
        }
        if (pMO->isDynamic() || pMO->isBad()) {
            vbNotIncludedMO[i] = true;
            continue;
        }


        g2o::VertexSE3Expmap *vSE3Obj = new g2o::VertexSE3Expmap();
        vSE3Obj->setEstimate(Converter::toSE3Quat(pMO->SE3Tow));
        int id = pMO->mnId + maxKFid + maxMPid + 2;
        vSE3Obj->setId(id);
        optimizer.addVertex(vSE3Obj);

        const map<KeyFrame *, size_t> observations = pMO->GetObservations();

        // Set Edges
        int nEdges = 0;
        for (auto observation : observations) {
            KeyFrame *pKFi = observation.first;

            // reject those frames after requesting stop
            if (pKFi->isBad() || pKFi->mnId > maxKFid)
                continue;
            // Get detections
            auto mvpObjectDetections = pKFi->GetObjectDetections();

            // cout << "Object KF ID: " << pKFi->mnId << endl;
            EdgeSE3LieAlgebra *e = new EdgeSE3LieAlgebra();  // BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>
            e->setVertex(0, optimizer.vertex(pKFi->mnId));
            e->setVertex(1, optimizer.vertex(id));
            auto det = mvpObjectDetections[observation.second];
            e->setMeasurement(Converter::toSE3Quat(det->SE3Tco));
            Eigen::Matrix<double, 6, 6> Info = Eigen::Matrix<double, 6, 6>::Identity();
            Info *= invSigmaObject;
            e->setInformation(Info);

            if (bRobust) {
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuberObject);
            }

            optimizer.addEdge(e);
            nEdges++;
        }

        if (nEdges == 0) {
            optimizer.removeVertex(vSE3Obj);
            vbNotIncludedMO[i] = true;
        } else {
            vbNotIncludedMO[i] = false;
        }

    }

    // Optimize!
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
            continue;
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if (nLoopKF == 0) {
            pKF->SetPose(Converter::toCvMat(SE3quat));
        } else {
            pKF->mTcwGBA.create(4, 4, CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    for (size_t i = 0; i < vpMP.size(); i++) {
        if (vbNotIncludedMP[i])
            continue;

        MapPoint *pMP = vpMP[i];

        if (pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
                pMP->mnId + maxKFid + 1));

        if (nLoopKF == 0) {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        } else {
            pMP->mPosGBA.create(3, 1, CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }

    // Objects
    for (size_t i = 0; i < vpMO.size(); i++) {
        if (vbNotIncludedMO[i])
            continue;

        MapObject *pMO = vpMO[i];

        if (pMO->isBad())
            continue;
        if (pMO->isDynamic())
            continue;

        g2o::VertexSE3Expmap *vSE3Obj = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(
                pMO->mnId + maxKFid + maxMPid + 2));
        g2o::SE3Quat SE3Tow = vSE3Obj->estimate();

        if (nLoopKF == 0) {
            Eigen::Matrix4f SE3Two = Converter::toMatrix4f(SE3Tow).inverse();
            pMO->SetObjectPoseSE3(SE3Two);
        } else {
            pMO->mTwoGBA = Converter::toMatrix4f(SE3Tow).inverse();
            pMO->mnBAGlobalForKF = nLoopKF;
        }
    }
}

void Optimizer::LocalJointBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap)
{
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

//    cout << "No. Local Keyframes: " << lLocalKeyFrames.size() << endl;

    // Local MapPoints and MapObjects seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;
    list<MapObject*> lLocalMapObjects;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if (pMP->mnBALocalForKF != pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
                }
            }
        }

        vector<MapObject*> vpMOs = (*lit)->GetMapObjectMatches();
        for (auto pMO : vpMOs)
        {
            if (pMO)
            {
                if (pMO->mnBALocalForKF != pKF->mnId)
                {
                    lLocalMapObjects.push_back(pMO);
                    pMO->mnBALocalForKF = pKF->mnId;
                }
            }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,tuple<int,int>> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;
    std::set<long unsigned int> msKeyframeIDs;

    // Set Local KeyFrame vertices
	cout << "Set Local KeyFrame vertices" << endl;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        msKeyframeIDs.insert(pKFi->mnId);
        // cout << "KF ID: " << pKFi->mnId << endl;
        vSE3->setFixed(pKFi->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
	cout << "Set Fixed KeyFrame vertices" << endl;
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        msKeyframeIDs.insert(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set MapPoint vertices and edges
	cout << "Set MapPoint vertices and edges" << endl;
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);
    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);
    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);
    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);
    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    vector<EdgeSE3LieAlgebra*> vpEdgesCamObj;
    vector<KeyFrame*> vpEdgeKFCamObj;
    vector<MapObject*> vpMapObjectEdgeCamObj;

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);
    const float invSigmaObject = 1e3;
    const float thHuberObjectSquare = 1e3;
    const float thHuberObject = sqrt(thHuberObjectSquare);

    unsigned long maxMPid = 0;

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();

        //Set edges
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad())
            {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[get<0>(mit->second)];

                // Monocular observation
                if(pKFi->mvuRight[get<0>(mit->second)]<0)
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);

                    if(pMP->mnId > maxMPid)
                        maxMPid = pMP->mnId;
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ(); // BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);

                    if(pMP->mnId > maxMPid)
                        maxMPid = pMP->mnId;
                }
            }
        }
    }

    // Set map object vertices and edges
	cout << "Set map object vertices and edges" << endl;
    for (auto pMO : lLocalMapObjects)
    {
        if (!pMO->isDynamic())
        {
            g2o::VertexSE3Expmap *vSE3Obj = new g2o::VertexSE3Expmap();
            vSE3Obj->setEstimate(Converter::toSE3Quat(pMO->SE3Tow));
            int id = pMO->mnId + maxKFid + maxMPid + 2;
            vSE3Obj->setId(id);
            optimizer.addVertex(vSE3Obj);

            const map<KeyFrame*, size_t> observations = pMO->GetObservations();

            for (auto observation : observations)
            {
                KeyFrame* pKFi = observation.first;
                if (msKeyframeIDs.count(pKFi->mnId) == 0)
                    continue;

                if(!pKFi->isBad())
                {
                    auto mvpObjectDetections = pKFi->GetObjectDetections();
                    // cout << "Object KF ID: " << pKFi->mnId << endl;
                    EdgeSE3LieAlgebra* e = new EdgeSE3LieAlgebra(); // BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>
                    e->setVertex(0, optimizer.vertex(pKFi->mnId));
                    e->setVertex(1, optimizer.vertex(id));
                    auto det = mvpObjectDetections[observation.second];
                    e->setMeasurement(Converter::toSE3Quat(det->SE3Tco));
                    Eigen::Matrix<double, 6, 6> Info = Eigen::Matrix<double, 6, 6>::Identity();
                    Info*= invSigmaObject;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberObject);

                    optimizer.addEdge(e);
                    vpEdgesCamObj.push_back(e);
                    vpEdgeKFCamObj.push_back(pKFi);
                    vpMapObjectEdgeCamObj.push_back(pMO);
                }
            }
        }
    }

    if(pbStopFlag)
    {
        if(*pbStopFlag)
        {
            // cout << "Local BA hasn't finished, but abort signal triggered!!!!!!!!!!!!!!!!!" << endl;
            return;
        }
    }
	
	cout << "Optimise" << endl;
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
    {
        if(*pbStopFlag)
        {
            // cout << "Local BA hasn't finished, but abort signal triggered!!!!!!!!!!!!!!!!!" << endl;
            return;
        }
    }

    if(bDoMore)
    {
        // Check inlier observations
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPoint* pMP = vpMapPointEdgeStereo[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesCamObj.size(); i<iend;i++)
        {
            auto e = vpEdgesCamObj[i];

            if(e->chi2() > thHuberObjectSquare)
            {
                e->setLevel(1);
            }
            e->setRobustKernel(0);
            // cout << "Edge " << vpEdgeKFCamObj[i]->mnId << "-" << vpMapObjectEdgeCamObj[i]->mnId << " Loss: " << e->chi2() << endl;
        }

        // Optimize again without the outliers
        optimizer.initializeOptimization(0);
		cout << "Optimise" << endl;
        optimizer.optimize(10);

    }

    vector<pair<KeyFrame*, MapPoint*>> vToEraseCamPoints;
    vector<pair<KeyFrame*, MapObject*>> vToEraseCamObjects;
    vToEraseCamPoints.reserve(vpEdgesMono.size() + vpEdgesStereo.size());
    vToEraseCamObjects.reserve(vpEdgesCamObj.size());

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToEraseCamPoints.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToEraseCamPoints.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesCamObj.size(); i<iend;i++)
    {
        auto e = vpEdgesCamObj[i];
        MapObject* pMO = vpMapObjectEdgeCamObj[i];

        if(e->chi2() > thHuberObjectSquare)
        {
            KeyFrame* pKFi = vpEdgeKFCamObj[i];
            vToEraseCamObjects.push_back(make_pair(pKFi, pMO));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

	cout << "Erase Cam Points" << endl;
    if(!vToEraseCamPoints.empty())
    {
        for(size_t i=0; i<vToEraseCamPoints.size(); i++)
        {
            KeyFrame* pKFi = vToEraseCamPoints[i].first;
            MapPoint* pMPi = vToEraseCamPoints[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

	cout << "Erase Cam Objects" << endl;
    if (!vToEraseCamObjects.empty())
    {
        for(size_t i=0; i<vToEraseCamObjects.size(); i++)
        {
            KeyFrame* pKFi = vToEraseCamObjects[i].first;
            MapObject* pMO = vToEraseCamObjects[i].second;
            pKFi->EraseMapObjectMatch(pMO);
            pMO->EraseObservation(pKFi);
        }
    }

    // Recover optimized data
	cout << "Recover Keyframes" << endl;
    //Keyframes
/*    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }
*/
	cout << "Recover Points" << endl;
    //Points
/*    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
*/
	cout << "Recover Objects" << endl;
    //Objects
    for (auto pMO : lLocalMapObjects)
    {
        if (!pMO->isDynamic() && !pMO->isBad())
        {
            g2o::VertexSE3Expmap* vSE3Obj = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pMO->mnId + maxKFid + maxMPid + 2));
            g2o::SE3Quat SE3Tow = vSE3Obj->estimate();
            Eigen::Matrix4f SE3Two = Converter::toMatrix4f(SE3Tow).inverse();
            pMO->SetObjectPoseSE3(SE3Two);
        }
    }
    Optimizer::nBAdone++;

}

// ORB_SLAM3
#ifdef COVINS_MOD
void Optimizer::LocalJointBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, list<KeyFrame *> &local_kfs)
#else
void Optimizer::LocalJointBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges)
#endif
{
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
	Map* pCurrentMap = pKF->GetMap();

    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

	#ifdef COVINS_MOD
    local_kfs = lLocalKeyFrames;
    #endif

//    cout << "No. Local Keyframes: " << lLocalKeyFrames.size() << endl;

    // Local MapPoints and MapObjects seen in Local KeyFrames
	num_fixedKF = 0;
    list<MapPoint*> lLocalMapPoints;
    list<MapObject*> lLocalMapObjects;
	set<MapPoint*> sNumObsMP;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
		if((*lit)->mnId==pMap->GetInitKFid())
        {
            num_fixedKF = 1;
        }
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
            {
                if(!pMP->isBad() && pMP->GetMap() == pCurrentMap)
                {
                    if (pMP->mnBALocalForKF != pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
                }
            }
        }

        vector<MapObject*> vpMOs = (*lit)->GetMapObjectMatches();
        for (auto pMO : vpMOs)
        {
            if (pMO)
            {
                if (pMO->mnBALocalForKF != pKF->mnId)
                {
                    lLocalMapObjects.push_back(pMO);
                    pMO->mnBALocalForKF = pKF->mnId;
                }
            }
        }
    }
	num_MPs = lLocalMapPoints.size();

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,tuple<int,int>> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                    lFixedCameras.push_back(pKFi);
            }
        }
    }
	num_fixedKF = lFixedCameras.size() + num_fixedKF;
    if(num_fixedKF < 2)
    {
        list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin();
        int lowerId = pKF->mnId;
        KeyFrame* pLowerKf;
        int secondLowerId = pKF->mnId;
        KeyFrame* pSecondLowerKF;

        for(; lit != lLocalKeyFrames.end(); lit++)
        {
            KeyFrame* pKFi = *lit;
            if(pKFi == pKF || pKFi->mnId == pMap->GetInitKFid())
            {
                continue;
            }

            if(pKFi->mnId < lowerId)
            {
                lowerId = pKFi->mnId;
                pLowerKf = pKFi;
            }
            else if(pKFi->mnId < secondLowerId)
            {
                secondLowerId = pKFi->mnId;
                pSecondLowerKF = pKFi;
            }
        }
        lFixedCameras.push_back(pLowerKf);
        lLocalKeyFrames.remove(pLowerKf);
        num_fixedKF++;
        if(num_fixedKF < 2)
        {
            lFixedCameras.push_back(pSecondLowerKF);
            lLocalKeyFrames.remove(pSecondLowerKF);
            num_fixedKF++;
        }
    }

    if(num_fixedKF == 0)
    {
        Verbose::PrintMess("LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted", Verbose::VERBOSITY_QUIET);
        return;
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    
	if (pCurrentMap->IsInertial())
        solver->setUserLambdaInit(100.0); // TODO uncomment

	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(false);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;
    std::set<long unsigned int> msKeyframeIDs;

    // Set Local KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        msKeyframeIDs.insert(pKFi->mnId);
        // cout << "KF ID: " << pKFi->mnId << endl;
        vSE3->setFixed(pKFi->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        msKeyframeIDs.insert(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set MapPoint vertices and edges
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;  // EdgesMono
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono; // EdgeKFMono
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono; // MapPointEdgeMono
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo; // EdgesStereo
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo; // EdgeKFStereo
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;  // MapPointEdgeStereo
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    vector<EdgeSE3LieAlgebra*> vpEdgesCamObj;
    vector<KeyFrame*> vpEdgeKFCamObj;
    vector<MapObject*> vpMapObjectEdgeCamObj;

	vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody*> vpEdgesBody; // EdgesBody
    vpEdgesBody.reserve(nExpectedSize);

	vector<KeyFrame*> vpEdgeKFBody; // EdgeKFBody
    vpEdgeKFBody.reserve(nExpectedSize);

	vector<MapPoint*> vpMapPointEdgeBody; // MapPointEdgeBody
    vpMapPointEdgeBody.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);
    const float invSigmaObject = 1e3;
    const float thHuberObjectSquare = 1e3;
    const float thHuberObject = sqrt(thHuberObjectSquare);

    unsigned long maxMPid = 0;

    int nKFs = lLocalKeyFrames.size()+lFixedCameras.size(), nEdges = 0, nPoints = 0;

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
		nPoints++;

        const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();

        //Set edges
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            {
				const int cam0Index = get<0>(mit->second);

                // Monocular observation
                if(cam0Index != -1 && pKFi->mvuRight[cam0Index]<0)
                {
					const cv::KeyPoint &kpUn = pKFi->mvKeysUn[cam0Index];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->pCamera = pKFi->mpCamera;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);

					nEdges++;

                    if(pMP->mnId > maxMPid)
                        maxMPid = pMP->mnId;
                }
				// Stereo observation
                else if(cam0Index != -1 && pKFi->mvuRight[cam0Index]>=0)
                {
					const cv::KeyPoint &kpUn = pKFi->mvKeysUn[cam0Index];
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ(); // BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);

					nEdges++;

                    if(pMP->mnId > maxMPid)
                        maxMPid = pMP->mnId;
                }
            }
        }
    }

    // Set map object vertices and edges
    for (auto pMO : lLocalMapObjects)
    {
        if (!pMO->isDynamic())
        {
            g2o::VertexSE3Expmap *vSE3Obj = new g2o::VertexSE3Expmap();
            vSE3Obj->setEstimate(Converter::toSE3Quat(pMO->SE3Tow));
            int id = pMO->mnId + maxKFid + maxMPid + 2;
            vSE3Obj->setId(id);
            optimizer.addVertex(vSE3Obj);

            const map<KeyFrame*, size_t> observations = pMO->GetObservations();

            for (auto observation : observations)
            {
                KeyFrame* pKFi = observation.first;
                if (msKeyframeIDs.count(pKFi->mnId) == 0)
                    continue;

                if(!pKFi->isBad())
                {
                    auto mvpObjectDetections = pKFi->GetObjectDetections();
                    // cout << "Object KF ID: " << pKFi->mnId << endl;
                    EdgeSE3LieAlgebra* e = new EdgeSE3LieAlgebra(); // BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>
                    e->setVertex(0, optimizer.vertex(pKFi->mnId));
                    e->setVertex(1, optimizer.vertex(id));
                    auto det = mvpObjectDetections[observation.second];
                    e->setMeasurement(Converter::toSE3Quat(det->SE3Tco));
                    Eigen::Matrix<double, 6, 6> Info = Eigen::Matrix<double, 6, 6>::Identity();
                    Info*= invSigmaObject;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberObject);

                    optimizer.addEdge(e);
                    vpEdgesCamObj.push_back(e);
                    vpEdgeKFCamObj.push_back(pKFi);
                    vpMapObjectEdgeCamObj.push_back(pMO);
                }
            }
        }
    }

    if(pbStopFlag)
    {
        if(*pbStopFlag)
        {
            // cout << "Local BA hasn't finished, but abort signal triggered!!!!!!!!!!!!!!!!!" << endl;
            return;
        }
    }

    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    int numPerform_it = optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
    {
        if(*pbStopFlag)
        {
            // cout << "Local BA hasn't finished, but abort signal triggered!!!!!!!!!!!!!!!!!" << endl;
            return;
        }
    }

    if(bDoMore)
    {
        // Check inlier observations
		int nMonoBadObs = 0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
				nMonoBadObs++;
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

		int nBodyBadObs = 0;
        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPoint* pMP = vpMapPointEdgeStereo[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
				nBodyBadObs++;
            }

            e->setRobustKernel(0);
        }

		int nObjBadObs = 0;
        for(size_t i=0, iend=vpEdgesCamObj.size(); i<iend;i++)
        {
            auto e = vpEdgesCamObj[i];

            if(e->chi2() > thHuberObjectSquare)
            {
                e->setLevel(1);
				nObjBadObs++;
            }
            e->setRobustKernel(0);
            // cout << "Edge " << vpEdgeKFCamObj[i]->mnId << "-" << vpMapObjectEdgeCamObj[i]->mnId << " Loss: " << e->chi2() << endl;
        }

        // Optimize again without the outliers
        optimizer.initializeOptimization(0);
        numPerform_it += optimizer.optimize(10);

    }

    vector<pair<KeyFrame*, MapPoint*>> vToEraseCamPoints;
    vector<pair<KeyFrame*, MapObject*>> vToEraseCamObjects;
    vToEraseCamPoints.reserve(vpEdgesMono.size()+vpEdgesBody.size()+vpEdgesStereo.size());
    vToEraseCamObjects.reserve(vpEdgesCamObj.size());

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToEraseCamPoints.push_back(make_pair(pKFi,pMP));
        }
    }

	for(size_t i=0, iend=vpEdgesBody.size(); i<iend;i++)
    {
        ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = vpEdgesBody[i];
        MapPoint* pMP = vpMapPointEdgeBody[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFBody[i];
            vToEraseCamPoints.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToEraseCamPoints.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesCamObj.size(); i<iend;i++)
    {
        auto e = vpEdgesCamObj[i];
        MapObject* pMO = vpMapObjectEdgeCamObj[i];

        if(e->chi2() > thHuberObjectSquare)
        {
            KeyFrame* pKFi = vpEdgeKFCamObj[i];
            vToEraseCamObjects.push_back(make_pair(pKFi, pMO));
        }
    }

	bool bRedrawError = false;
    bool bWriteStats = false;

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if(!vToEraseCamPoints.empty())
    {
        for(size_t i=0; i<vToEraseCamPoints.size(); i++)
        {
            KeyFrame* pKFi = vToEraseCamPoints[i].first;
            MapPoint* pMPi = vToEraseCamPoints[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    if (!vToEraseCamObjects.empty())
    {
        for(size_t i=0; i<vToEraseCamObjects.size(); i++)
        {
            KeyFrame* pKFi = vToEraseCamObjects[i].first;
            MapObject* pMO = vToEraseCamObjects[i].second;
            pKFi->EraseMapObjectMatch(pMO);
            pMO->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    //Objects
    for (auto pMO : lLocalMapObjects)
    {
        if (!pMO->isDynamic() && !pMO->isBad())
        {
            g2o::VertexSE3Expmap* vSE3Obj = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pMO->mnId + maxKFid + maxMPid + 2));
            g2o::SE3Quat SE3Tow = vSE3Obj->estimate();
            Eigen::Matrix4f SE3Two = Converter::toMatrix4f(SE3Tow).inverse();
            pMO->SetObjectPoseSE3(SE3Two);
        }
    }

	pCurrentMap->IncreaseChangeIndex();
    Optimizer::nBAdone++;

}

#ifdef COVINS_MOD
void Optimizer::LocalJointInertialBA(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, vector<KeyFrame *> &local_kfs, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge, bool bRecInit)
#else
void Optimizer::LocalJointInertialBA(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge, bool bRecInit)
#endif
{
    Map* pCurrentMap = pKF->GetMap();

    int maxOpt=10;
    int opt_it=10;
    if(bLarge)
    {
        maxOpt=25;
        opt_it=4;
    }
    const int Nd = std::min((int)pCurrentMap->KeyFramesInMap()-2,maxOpt);
    const unsigned long maxKFid = pKF->mnId;

    vector<KeyFrame*> vpOptimizableKFs;
    const vector<KeyFrame*> vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
    list<KeyFrame*> lpOptVisKFs;

    vpOptimizableKFs.reserve(Nd);
    vpOptimizableKFs.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
    for(int i=1; i<Nd; i++)
    {
        if(vpOptimizableKFs.back()->mPrevKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
        }
        else
            break;
    }

    int N = vpOptimizableKFs.size();

    #ifdef COVINS_MOD
    local_kfs = vpOptimizableKFs;
    #endif

    // Optimizable points and objects seen by temporal optimizable keyframes
    list<MapPoint*> lLocalMapPoints;
	list<MapObject*> lLocalMapObjects;
    for(int i=0; i<N; i++)
    {
		// points
        vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
        }
		// objects
		vector<MapObject*> vpMOs = vpOptimizableKFs[i]->GetMapObjectMatches();
        for (auto pMO : vpMOs)
        {
            if (pMO)
            {
                if (pMO->mnBALocalForKF != pKF->mnId)
                {
                    lLocalMapObjects.push_back(pMO);
                    pMO->mnBALocalForKF = pKF->mnId;
                }
            }
        }
    }

    // Fixed Keyframe: First frame previous KF to optimization window)
    list<KeyFrame*> lFixedKeyFrames;
    if(vpOptimizableKFs.back()->mPrevKF)
    {
        lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
        vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF=pKF->mnId;
    }
    else
    {
        vpOptimizableKFs.back()->mnBALocalForKF=0;
        vpOptimizableKFs.back()->mnBAFixedForKF=pKF->mnId;
        lFixedKeyFrames.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Optimizable visual KFs
    const int maxCovKF = 0;
    for(int i=0, iend=vpNeighsKFs.size(); i<iend; i++)
    {
        if(lpOptVisKFs.size() >= maxCovKF)
            break;

        KeyFrame* pKFi = vpNeighsKFs[i];
        if(pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId)
            continue;
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
        {
            lpOptVisKFs.push_back(pKFi);

			// points
            vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
            for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
            {
                MapPoint* pMP = *vit;
                if(pMP)
                    if(!pMP->isBad())
                        if(pMP->mnBALocalForKF!=pKF->mnId)
                        {
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF=pKF->mnId;
                        }
            }

			// objects
			vector<MapObject*> vpMOs = pKFi->GetMapObjectMatches();
		    for (auto pMO : vpMOs)
		    {
		        if (pMO)
		        {
		            if (pMO->mnBALocalForKF != pKF->mnId)
		            {
		                lLocalMapObjects.push_back(pMO);
		                pMO->mnBALocalForKF = pKF->mnId;
		            }
		        }
		    }
        }
    }

    // Fixed KFs which are not covisible optimizable
    const int maxFixKF = 200;
	std::set<long unsigned int> msKeyframeIDs;

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,tuple<int,int>> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                {
                    lFixedKeyFrames.push_back(pKFi);
                    break;
                }
            }
        }
        if(lFixedKeyFrames.size()>=maxFixKF)
            break;
    }

    bool bNonFixed = (lFixedKeyFrames.size() == 0);

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    if(bLarge)
    {
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(1e-2); // to avoid iterating for finding optimal lambda
        optimizer.setAlgorithm(solver);
    }
    else
    {
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(1e0);
        optimizer.setAlgorithm(solver);
    }


    // Set Local temporal KeyFrame vertices
    N=vpOptimizableKFs.size();
    num_fixedKF = 0;
    num_OptKF = 0;
    num_MPs = 0;
    num_edges = 0;
    for(int i=0; i<N; i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        VertexPose * VP = new VertexPose(pKFi);  // public g2o::BaseVertex<6,ImuCamPose>
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        if(pKFi->bImu)
        {
            VertexVelocity* VV = new VertexVelocity(pKFi); // public g2o::BaseVertex<3,Eigen::Vector3d>
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(false);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi); // public g2o::BaseVertex<3,Eigen::Vector3d>
            VG->setId(maxKFid+3*(pKFi->mnId)+2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi); // public g2o::BaseVertex<3,Eigen::Vector3d>
            VA->setId(maxKFid+3*(pKFi->mnId)+3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }
        num_OptKF++;
    }

    // Set Local visual KeyFrame vertices
    for(list<KeyFrame*>::iterator it=lpOptVisKFs.begin(), itEnd = lpOptVisKFs.end(); it!=itEnd; it++)
    {
        KeyFrame* pKFi = *it;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        num_OptKF++;
    }

    // Set Fixed KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        if(pKFi->bImu) // This should be done only for keyframe just before temporal window
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(true);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid+3*(pKFi->mnId)+2);
            VG->setFixed(true);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid+3*(pKFi->mnId)+3);
            VA->setFixed(true);
            optimizer.addVertex(VA);
        }
        num_fixedKF++;
    }

    // Create intertial constraints
    vector<EdgeInertial*> vei(N,(EdgeInertial*)NULL);
    vector<EdgeGyroRW*> vegr(N,(EdgeGyroRW*)NULL);
    vector<EdgeAccRW*> vear(N,(EdgeAccRW*)NULL);

    for(int i=0;i<N;i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        if(!pKFi->mPrevKF)
        {
            cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << endl;
            continue;
        }
        if(pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated)
        {
            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+1);
            g2o::HyperGraph::Vertex* VG1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+2);
            g2o::HyperGraph::Vertex* VA1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+3);
            g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+1);
            g2o::HyperGraph::Vertex* VG2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+2);
            g2o::HyperGraph::Vertex* VA2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+3);

            if(!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
            {
                cerr << "Error " << VP1 << ", "<< VV1 << ", "<< VG1 << ", "<< VA1 << ", " << VP2 << ", " << VV2 <<  ", "<< VG2 << ", "<< VA2 <<endl;
                continue;
            }

            vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);  // public g2o::BaseMultiEdge<9,Vector9d>

            vei[i]->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            vei[i]->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            vei[i]->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
            vei[i]->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
            vei[i]->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            vei[i]->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

            if(i==N-1 || bRecInit)
            {
                // All inertial residuals are included without robust cost function, but not that one linking the
                // last optimizable keyframe inside of the local window and the first fixed keyframe out. The
                // information matrix for this measurement is also downweighted. This is done to avoid accumulating
                // error due to fixing variables.
                g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
                vei[i]->setRobustKernel(rki);
                if(i==N-1)
                    vei[i]->setInformation(vei[i]->information()*1e-2);
                rki->setDelta(sqrt(16.92));
            }
            optimizer.addEdge(vei[i]);

            vegr[i] = new EdgeGyroRW();
            vegr[i]->setVertex(0,VG1);
            vegr[i]->setVertex(1,VG2);
            cv::Mat cvInfoG = pKFi->mpImuPreintegrated->C.rowRange(9,12).colRange(9,12).inv(cv::DECOMP_SVD);
            Eigen::Matrix3d InfoG;

            for(int r=0;r<3;r++)
                for(int c=0;c<3;c++)
                    InfoG(r,c)=cvInfoG.at<float>(r,c);
            vegr[i]->setInformation(InfoG);
            optimizer.addEdge(vegr[i]);
            num_edges++;

            vear[i] = new EdgeAccRW();
            vear[i]->setVertex(0,VA1);
            vear[i]->setVertex(1,VA2);
            cv::Mat cvInfoA = pKFi->mpImuPreintegrated->C.rowRange(12,15).colRange(12,15).inv(cv::DECOMP_SVD);
            Eigen::Matrix3d InfoA;
            for(int r=0;r<3;r++)
                for(int c=0;c<3;c++)
                    InfoA(r,c)=cvInfoA.at<float>(r,c);
            vear[i]->setInformation(InfoA);           

            optimizer.addEdge(vear[i]);
            num_edges++;
        }
        else
            cout << "ERROR building inertial edge" << endl;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (N+lFixedKeyFrames.size())*lLocalMapPoints.size();

    // Mono
    vector<EdgeMono*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    // Stereo
    vector<EdgeStereo*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

	// Object
	vector<EdgeSE3LieAlgebra_2*> vpEdgesCamObj;  // A new edge that maps VertexSE3Expmap to VertexPose
    vector<KeyFrame*> vpEdgeKFCamObj;
    vector<MapObject*> vpMapObjectEdgeCamObj;

    const float thHuberMono = sqrt(5.991);
    const float chi2Mono2 = 5.991;
    const float thHuberStereo = sqrt(7.815);
    const float chi2Stereo2 = 7.815;

	// Object
	const float thHuberObjectSquare = 1e3;
    const float thHuberObject = sqrt(thHuberObjectSquare); 
    const float invSigmaObject = 1e3;

	long unsigned int maxMPid = 0;
    const unsigned long iniMPid = maxKFid*5;

    map<int,int> mVisEdges;
    for(int i=0;i<N;i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];
        mVisEdges[pKFi->mnId] = 0;
    }
    for(list<KeyFrame*>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
    {
        mVisEdges[(*lit)->mnId] = 0;
    }

    num_MPs = lLocalMapPoints.size();
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));

        unsigned long id = pMP->mnId+iniMPid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();

        // Create visual constraints
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
                continue;

            if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            {
                const int leftIndex = get<0>(mit->second);

                cv::KeyPoint kpUn;

                // Monocular left observation
                if(leftIndex != -1 && pKFi->mvuRight[leftIndex]<0)
                {
                    mVisEdges[pKFi->mnId]++;

                    kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono* e = new EdgeMono(0);

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);

                    num_edges++;
                }
                // Stereo-observation
                else if(leftIndex != -1)// Stereo observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    mVisEdges[pKFi->mnId]++;

                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    Eigen::Matrix<double,3,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo* e = new EdgeStereo(0); // public g2o::BaseBinaryEdge<3,Eigen::Vector3d,g2o::VertexSBAPointXYZ,VertexPose>

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pKFi->mpCamera->uncertainty2(obs.head(2));

                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);

                    num_edges++;
                }

                // Monocular right observation
                if(pKFi->mpCamera2){
                    int rightIndex = get<1>(mit->second);

                    if(rightIndex != -1 ){
                        rightIndex -= pKFi->NLeft;
                        mVisEdges[pKFi->mnId]++;

                        Eigen::Matrix<double,2,1> obs;
                        cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        EdgeMono* e = new EdgeMono(1);

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);

                        num_edges++;
                    }
                }
            }
        }
    }

    //cout << "Total map points: " << lLocalMapPoints.size() << endl;
    for(map<int,int>::iterator mit=mVisEdges.begin(), mend=mVisEdges.end(); mit!=mend; mit++)
    {
        assert(mit->second>=3);
    }

	// Set map object vertices and edges
    for (auto pMO : lLocalMapObjects)
    {
        if (!pMO->isDynamic())
        {
            g2o::VertexSE3Expmap *vSE3Obj = new g2o::VertexSE3Expmap();
            vSE3Obj->setEstimate(Converter::toSE3Quat(pMO->SE3Tow));
            int id = pMO->mnId + iniMPid + maxMPid + 2;
            vSE3Obj->setId(id);
            optimizer.addVertex(vSE3Obj);

            const map<KeyFrame*, size_t> observations = pMO->GetObservations();

            for (auto observation : observations)
            {
                KeyFrame* pKFi = observation.first;
                if (msKeyframeIDs.count(pKFi->mnId) == 0)
                    continue;

                if(!pKFi->isBad())
                {
                    auto mvpObjectDetections = pKFi->GetObjectDetections();
                    // cout << "Object KF ID: " << pKFi->mnId << endl;
                    EdgeSE3LieAlgebra_2* e = new EdgeSE3LieAlgebra_2();  // public g2o::BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexPose>
                    e->setVertex(0, optimizer.vertex(pKFi->mnId));
                    e->setVertex(1, optimizer.vertex(id));
                    auto det = mvpObjectDetections[observation.second];
                    e->setMeasurement(Converter::toSE3Quat(det->SE3Tco));
                    Eigen::Matrix<double, 6, 6> Info = Eigen::Matrix<double, 6, 6>::Identity();
                    Info*= invSigmaObject;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberObject);

                    optimizer.addEdge(e);
                    vpEdgesCamObj.push_back(e);
                    vpEdgeKFCamObj.push_back(pKFi);
                    vpMapObjectEdgeCamObj.push_back(pMO);
                }
            }
        }
    }

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();

    float err = optimizer.activeRobustChi2();
    optimizer.optimize(opt_it); // Originally to 2
    float err_end = optimizer.activeRobustChi2();

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());
	
	vector<pair<KeyFrame*, MapObject*>> vToEraseCamObjects;
	vToEraseCamObjects.reserve(vpEdgesCamObj.size());

    // Check inlier observations
    // Mono
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        EdgeMono* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];
        bool bClose = pMP->mTrackDepth<10.f;

        if(pMP->isBad())
            continue;

        if((e->chi2()>chi2Mono2 && !bClose) || (e->chi2()>1.5f*chi2Mono2 && bClose) || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }


    // Stereo
    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        EdgeStereo* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>chi2Stereo2)
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

	// Objects
	for(size_t i=0, iend=vpEdgesCamObj.size(); i<iend;i++)
    {
        auto e = vpEdgesCamObj[i];
        MapObject* pMO = vpMapObjectEdgeCamObj[i];

		if(pMO->isBad())
            continue;

        if(e->chi2() > thHuberObjectSquare)
        {
            KeyFrame* pKFi = vpEdgeKFCamObj[i];
            vToEraseCamObjects.push_back(make_pair(pKFi, pMO));
        }
    }

    // Get Map Mutex and erase outliers
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if((2*err < err_end || isnan(err) || isnan(err_end)) && !bLarge)
    {
        cout << "FAIL LOCAL-INERTIAL BA!!!!" << endl;
        return;
    }



    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

	// Objects
	if (!vToEraseCamObjects.empty())
    {
        for(size_t i=0; i<vToEraseCamObjects.size(); i++)
        {
            KeyFrame* pKFi = vToEraseCamObjects[i].first;
            MapObject* pMO = vToEraseCamObjects[i].second;
            pKFi->EraseMapObjectMatch(pMO);
            pMO->EraseObservation(pKFi);
        }
    }

    // Display main statistcis of optimization
    //Verbose::PrintMess("LIBA KFs: " + to_string(N), Verbose::VERBOSITY_DEBUG);
    //Verbose::PrintMess("LIBA bNonFixed?: " + to_string(bNonFixed), Verbose::VERBOSITY_DEBUG);
    //Verbose::PrintMess("LIBA KFs visual outliers: " + to_string(vToErase.size()), Verbose::VERBOSITY_DEBUG);

    for(list<KeyFrame*>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
        (*lit)->mnBAFixedForKF = 0;

    // Recover optimized data
    // Local temporal Keyframes
    N=vpOptimizableKFs.size();
    for(int i=0; i<N; i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        cv::Mat Tcw = Converter::toCvSE3(VP->estimate().Rcw[0], VP->estimate().tcw[0]);
        pKFi->SetPose(Tcw);
        pKFi->mnBALocalForKF=0;

        if(pKFi->bImu)
        {
            VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+1));
            pKFi->SetVelocity(Converter::toCvMat(VV->estimate()));
            VertexGyroBias* VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+2));
            VertexAccBias* VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+3));
            Vector6d b;
            b << VG->estimate(), VA->estimate();
            pKFi->SetNewBias(IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]));

        }
    }

    // Local visual KeyFrame
    for(list<KeyFrame*>::iterator it=lpOptVisKFs.begin(), itEnd = lpOptVisKFs.end(); it!=itEnd; it++)
    {
        KeyFrame* pKFi = *it;
        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        cv::Mat Tcw = Converter::toCvSE3(VP->estimate().Rcw[0], VP->estimate().tcw[0]);
        pKFi->SetPose(Tcw);
        pKFi->mnBALocalForKF=0;
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+iniMPid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

	//Objects
    for (auto pMO : lLocalMapObjects)
    {
        if (!pMO->isDynamic() && !pMO->isBad())
        {
            g2o::VertexSE3Expmap* vSE3Obj = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pMO->mnId + iniMPid + maxMPid + 2));
            g2o::SE3Quat SE3Tow = vSE3Obj->estimate();
            Eigen::Matrix4f SE3Two = Converter::toMatrix4f(SE3Tow).inverse();
            pMO->SetObjectPoseSE3(SE3Two);
        }
    }

    pMap->IncreaseChangeIndex();

}

void Optimizer::FullInertialJointBA(Map *pMap, int its, const bool bFixLocal, const long unsigned int nLoopId, bool *pbStopFlag, bool bInit, float priorG, float priorA, Eigen::VectorXd *vSingVal, bool *bHess)
{

    long unsigned int maxKFid = pMap->GetMaxKFid();
    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();
	const vector<MapObject*> vpMOs = pMap->GetAllMapObjects();// 

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e-5);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    int nNonFixed = 0;

    // Set KeyFrame vertices
    KeyFrame* pIncKF;
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        pIncKF=pKFi;
        bool bFixed = false;
        if(bFixLocal)
        {
            bFixed = (pKFi->mnBALocalForKF>=(maxKFid-1)) || (pKFi->mnBAFixedForKF>=(maxKFid-1));
            if(!bFixed)
                nNonFixed++;
            VP->setFixed(bFixed);
        }
        optimizer.addVertex(VP);

        if(pKFi->bImu)
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(bFixed);
            optimizer.addVertex(VV);
            if (!bInit)
            {
                VertexGyroBias* VG = new VertexGyroBias(pKFi);
                VG->setId(maxKFid+3*(pKFi->mnId)+2);
                VG->setFixed(bFixed);
                optimizer.addVertex(VG);
                VertexAccBias* VA = new VertexAccBias(pKFi);
                VA->setId(maxKFid+3*(pKFi->mnId)+3);
                VA->setFixed(bFixed);
                optimizer.addVertex(VA);
            }
        }
    }

    if (bInit)
    {
        VertexGyroBias* VG = new VertexGyroBias(pIncKF);
        VG->setId(4*maxKFid+2);
        VG->setFixed(false);
        optimizer.addVertex(VG);
        VertexAccBias* VA = new VertexAccBias(pIncKF);
        VA->setId(4*maxKFid+3);
        VA->setFixed(false);
        optimizer.addVertex(VA);
    }

    if(bFixLocal)
    {
        if(nNonFixed<3)
            return;
    }

    // IMU links
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        if(!pKFi->mPrevKF)
        {
            Verbose::PrintMess("NOT INERTIAL LINK TO PREVIOUS FRAME!", Verbose::VERBOSITY_NORMAL);
            continue;
        }

        if(pKFi->mPrevKF && pKFi->mnId<=maxKFid)
        {
            if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
                continue;
            if(pKFi->bImu && pKFi->mPrevKF->bImu)
            {
                pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
                g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+1);

                g2o::HyperGraph::Vertex* VG1;
                g2o::HyperGraph::Vertex* VA1;
                g2o::HyperGraph::Vertex* VG2;
                g2o::HyperGraph::Vertex* VA2;
                if (!bInit)
                {
                    VG1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+2);
                    VA1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+3);
                    VG2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+2);
                    VA2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+3);
                }
                else
                {
                    VG1 = optimizer.vertex(4*maxKFid+2);
                    VA1 = optimizer.vertex(4*maxKFid+3);
                }

                g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
                g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+1);

                if (!bInit)
                {
                    if(!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
                    {
                        cout << "Error" << VP1 << ", "<< VV1 << ", "<< VG1 << ", "<< VA1 << ", " << VP2 << ", " << VV2 <<  ", "<< VG2 << ", "<< VA2 <<endl;

                        continue;
                    }
                }
                else
                {
                    if(!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2)
                    {
                        cout << "Error" << VP1 << ", "<< VV1 << ", "<< VG1 << ", "<< VA1 << ", " << VP2 << ", " << VV2 <<endl;

                        continue;
                    }
                }

                EdgeInertial* ei = new EdgeInertial(pKFi->mpImuPreintegrated);
                ei->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
                ei->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
                ei->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
                ei->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
                ei->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
                ei->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

                g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
                ei->setRobustKernel(rki);
                rki->setDelta(sqrt(16.92));

                optimizer.addEdge(ei);

                if (!bInit)
                {
                    EdgeGyroRW* egr= new EdgeGyroRW();
                    egr->setVertex(0,VG1);
                    egr->setVertex(1,VG2);
                    cv::Mat cvInfoG = pKFi->mpImuPreintegrated->C.rowRange(9,12).colRange(9,12).inv(cv::DECOMP_SVD);
                    Eigen::Matrix3d InfoG;
                    for(int r=0;r<3;r++)
                        for(int c=0;c<3;c++)
                            InfoG(r,c)=cvInfoG.at<float>(r,c);
                    egr->setInformation(InfoG);
                    egr->computeError();
                    optimizer.addEdge(egr);

                    EdgeAccRW* ear = new EdgeAccRW();
                    ear->setVertex(0,VA1);
                    ear->setVertex(1,VA2);
                    cv::Mat cvInfoA = pKFi->mpImuPreintegrated->C.rowRange(12,15).colRange(12,15).inv(cv::DECOMP_SVD);
                    Eigen::Matrix3d InfoA;
                    for(int r=0;r<3;r++)
                        for(int c=0;c<3;c++)
                            InfoA(r,c)=cvInfoA.at<float>(r,c);
                    ear->setInformation(InfoA);
                    ear->computeError();
                    optimizer.addEdge(ear);
                }
            }
            else
            {
                cout << pKFi->mnId << " or " << pKFi->mPrevKF->mnId << " no imu" << endl;
            }
        }
    }

    if (bInit)
    {
        g2o::HyperGraph::Vertex* VG = optimizer.vertex(4*maxKFid+2);
        g2o::HyperGraph::Vertex* VA = optimizer.vertex(4*maxKFid+3);

        // Add prior to comon biases
        EdgePriorAcc* epa = new EdgePriorAcc(cv::Mat::zeros(3,1,CV_32F));
        epa->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
        double infoPriorA = priorA; //
        epa->setInformation(infoPriorA*Eigen::Matrix3d::Identity());
        optimizer.addEdge(epa);

        EdgePriorGyro* epg = new EdgePriorGyro(cv::Mat::zeros(3,1,CV_32F));
        epg->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
        double infoPriorG = priorG; //
        epg->setInformation(infoPriorG*Eigen::Matrix3d::Identity());
        optimizer.addEdge(epg);
    }

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);
	const float invSigmaObject = 1e3;
    const float thHuberObject = sqrt(0.10 * invSigmaObject);
    const float thHuberObjectSquare = pow(thHuberObject, 2);

    const unsigned long iniMPid = maxKFid*5; // FIXME WHY 5? should be (4*maxKFid+3)+1?
	//const unsigned long iniMPid = 4*maxKFid+3;

	long unsigned int maxMPid = 0;

    vector<bool> vbNotIncludedMP(vpMPs.size(),false);

	// Set MapPoint vertices
    for(size_t i=0; i<vpMPs.size(); i++)
    {
        MapPoint* pMP = vpMPs[i];
        if (pMP->isBad()) // TARIQ ADDED
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        unsigned long id = pMP->mnId + iniMPid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
		if (pMP->mnId>maxMPid)
			maxMPid=pMP->mnId;

        const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();


        bool bAllFixed = true;

        //Set edges
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->isBad() || pKFi->mnId>maxKFid)
                continue;

            if(!pKFi->isBad())
            {
                const int leftIndex = get<0>(mit->second);
                cv::KeyPoint kpUn;

                if(leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)]<0) // Monocular observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono* e = new EdgeMono(0);

                    g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                    if(bAllFixed)
                        if(!VP->fixed())
                            bAllFixed=false;

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, VP);
                    e->setMeasurement(obs);
                    const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);
                }
                else if(leftIndex != -1 && pKFi->mvuRight[leftIndex] >= 0) // stereo observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    Eigen::Matrix<double,3,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo* e = new EdgeStereo(0);

                    g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                    if(bAllFixed)
                        if(!VP->fixed())
                            bAllFixed=false;

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, VP);
                    e->setMeasurement(obs);
                    const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                    e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                }

                if(pKFi->mpCamera2){ // Monocular right observation
                    int rightIndex = get<1>(mit->second);

                    if(rightIndex != -1 && rightIndex < pKFi->mvKeysRight.size()){
                        rightIndex -= pKFi->NLeft;

                        Eigen::Matrix<double,2,1> obs;
                        kpUn = pKFi->mvKeysRight[rightIndex];
                        obs << kpUn.pt.x, kpUn.pt.y;

                        EdgeMono *e = new EdgeMono(1);

                        g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                        if(bAllFixed)
                            if(!VP->fixed())
                                bAllFixed=false;

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        e->setVertex(1, VP);
                        e->setMeasurement(obs);
                        const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);
                    }
                }
            }
        }

        if(bAllFixed)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
    }

	// Set MapObject Vertices - Object-SLAM
	vector<bool> vbNotIncludedMO(vpMOs.size(), false);
    for (size_t i = 0; i < vpMOs.size(); i++) {
        auto pMO = vpMOs[i];

        if (!pMO) {
            vbNotIncludedMO[i] = true;
            continue;
        }
        if (pMO->isDynamic() || pMO->isBad()) {
            vbNotIncludedMO[i] = true;
            continue;
        }

        g2o::VertexSE3Expmap *vSE3Obj = new g2o::VertexSE3Expmap();
        vSE3Obj->setEstimate(Converter::toSE3Quat(pMO->SE3Tow));
        int id = pMO->mnId + iniMPid + maxMPid + 2;
        vSE3Obj->setId(id);
        optimizer.addVertex(vSE3Obj);

        const map<KeyFrame *, size_t> observations = pMO->GetObservations();

        // Set Edges
        int nEdges = 0;
        for (auto observation : observations) {
            KeyFrame *pKFi = observation.first;

            // reject those frames after requesting stop
            if (pKFi->isBad() || pKFi->mnId > maxKFid)
                continue;
            // Get detections
            auto mvpObjectDetections = pKFi->GetObjectDetections();
			auto det = mvpObjectDetections[observation.second];

            // cout << "Object KF ID: " << pKFi->mnId << endl;
            EdgeSE3LieAlgebra_2 *e = new EdgeSE3LieAlgebra_2(); // public g2o::BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexPose>
			g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
            e->setVertex(0, VP);
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setMeasurement(Converter::toSE3Quat(det->SE3Tco));
            Eigen::Matrix<double, 6, 6> Info = Eigen::Matrix<double, 6, 6>::Identity();
            Info *= invSigmaObject;
            e->setInformation(Info);

            //if (bRobust) {
			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(thHuberObject);
            //}

            optimizer.addEdge(e);
            nEdges++;
        }

        if (nEdges == 0) {
            optimizer.removeVertex(vSE3Obj);
            vbNotIncludedMO[i] = true;
        } else {
            vbNotIncludedMO[i] = false;
        }

    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(its);

    // Recover optimized data
    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->isBad() || pKFi->mnId>maxKFid)
            continue;
        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        if(nLoopId==0)
        {
            cv::Mat Tcw = Converter::toCvSE3(VP->estimate().Rcw[0], VP->estimate().tcw[0]);
            pKFi->SetPose(Tcw);
        }
        else
        {
            pKFi->mTcwGBA = cv::Mat::eye(4,4,CV_32F);
            Converter::toCvMat(VP->estimate().Rcw[0]).copyTo(pKFi->mTcwGBA.rowRange(0,3).colRange(0,3));
            Converter::toCvMat(VP->estimate().tcw[0]).copyTo(pKFi->mTcwGBA.rowRange(0,3).col(3));
			pKFi->mnBAGlobalForKF = nLoopId;

        }
        if(pKFi->bImu)
        {
            VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+1));
            if(nLoopId==0)
            {
                pKFi->SetVelocity(Converter::toCvMat(VV->estimate()));
            }
            else
            {
                pKFi->mVwbGBA = Converter::toCvMat(VV->estimate());
            }

            VertexGyroBias* VG;
            VertexAccBias* VA;
            if (!bInit)
            {
                VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+2));
                VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+3));
            }
            else
            {
                VG = static_cast<VertexGyroBias*>(optimizer.vertex(4*maxKFid+2));
                VA = static_cast<VertexAccBias*>(optimizer.vertex(4*maxKFid+3));
            }

            Vector6d vb;
            vb << VG->estimate(), VA->estimate();
            IMU::Bias b (vb[3],vb[4],vb[5],vb[0],vb[1],vb[2]);
            if(nLoopId==0)
            {
                pKFi->SetNewBias(b);
            }
            else
            {
                pKFi->mBiasGBA = b;
            }
        }
    }

    //Points
    for(size_t i=0; i<vpMPs.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMPs[i];
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+iniMPid+1));

        if(nLoopId==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopId;
        }

    }

	// Objects
    for (size_t i = 0; i < vpMOs.size(); i++) 
	{
        if (vbNotIncludedMO[i])
            continue;

        MapObject *pMO = vpMOs[i];

        if (pMO->isBad())
            continue;
        if (pMO->isDynamic())
            continue;

        g2o::VertexSE3Expmap* vSE3Obj = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pMO->mnId + iniMPid + maxMPid + 2));
        g2o::SE3Quat SE3Tow = vSE3Obj->estimate();

        if (nLoopId == 0) {
            Eigen::Matrix4f SE3Two = Converter::toMatrix4f(SE3Tow).inverse();
            pMO->SetObjectPoseSE3(SE3Two);
        } else {
            pMO->mTwoGBA = Converter::toMatrix4f(SE3Tow).inverse();
            pMO->mnBAGlobalForKF = nLoopId;
        }

    }

    pMap->IncreaseChangeIndex();
}

}
