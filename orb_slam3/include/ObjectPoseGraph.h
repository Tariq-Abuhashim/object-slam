
/**
* Tariq updated - Oct, 2022
* Tariq updated - Jan, 2023
**/

#ifndef OBJECTPOSEGRAPH_H
#define OBJECTPOSEGRAPH_H

#include <Eigen/Core>
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include <Eigen/Geometry>
#include "G2oTypes.h"
#include "Converter.h"

namespace ORB_SLAM3 {

using namespace g2o;
using namespace std;

class VertexSE3Object : public BaseVertex<6, SE3Quat> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSE3Object() {};

    virtual bool read(std::istream &is) {
        return true;
    }

    virtual bool write(std::ostream &os) const {
        return os.good();
    }

    virtual void setToOriginImpl() {
        _estimate = SE3Quat();
    }

    // gradient is wrt Twc, but our estimate is Tcw
    virtual void oplusImpl(const double *update_) {
        Eigen::Map<Vector6d> update(const_cast<double *>(update_));
        SE3Quat s(update);
        setEstimate(estimate() * s.inverse());
    }
};

class EdgeSE3LieAlgebra : public BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool read(std::istream &is) {
        return true;
    }

    bool write(std::ostream &os) const {
        return os.good();
    }

    virtual void computeError() {
        SE3Quat v1 = (static_cast<VertexSE3Expmap *>(_vertices[0]))->estimate(); // Ti
        SE3Quat v2 = (static_cast<VertexSE3Expmap *>(_vertices[1]))->estimate(); // Tj
        _error = (_measurement.inverse() * v1 * v2.inverse()).log(); // Tij^-1 * Ti * Tj^-1
    }

    virtual void linearizeOplus() {
        Matrix6d J;
        Eigen::Vector3d t, w;
        w = _error.head<3>();
        t = _error.tail<3>();
        J.block<3, 3>(0, 0) = skew(w);
        J.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
        J.block<3, 3>(3, 0) = skew(t);
        J.block<3, 3>(3, 3) = skew(w);
        J = 0.5 * J + Matrix6d::Identity();

        _jacobianOplusXi = J * _measurement.inverse().adj();
        _jacobianOplusXj = -J;
    }
};

// Tariq
// This class was created to work with ORBSLAM3 VertexPose
class EdgeSE3LieAlgebra_2 : public BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexPose> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool read(std::istream &is) {
        return true;
    }

    bool write(std::ostream &os) const {
        return os.good();
    }

    virtual void computeError() {
		const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
		cv::Mat Tcw = Converter::toCvSE3(VPose->estimate().Rcw[0], VPose->estimate().tcw[0]);
		SE3Quat v1 = Converter::toSE3Quat(Tcw);
        //SE3Quat v1 = (static_cast<VertexSE3Expmap *>(_vertices[0]))->estimate(); // Ti
        SE3Quat v2 = (static_cast<VertexSE3Expmap *>(_vertices[1]))->estimate(); // Tj
        _error = (_measurement.inverse() * v1 * v2.inverse()).log(); // Tij^-1 * Ti * Tj^-1
    }

    virtual void linearizeOplus() {
        Matrix6d J;
        Eigen::Vector3d t, w;
        w = _error.head<3>();
        t = _error.tail<3>();
        J.block<3, 3>(0, 0) = skew(w);
        J.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
        J.block<3, 3>(3, 0) = skew(t);
        J.block<3, 3>(3, 3) = skew(w);
        J = 0.5 * J + Matrix6d::Identity();

        _jacobianOplusXi = J * _measurement.inverse().adj();
        _jacobianOplusXj = -J;
    }
};

}
#endif //OBJECTPOSEGRAPH_H
