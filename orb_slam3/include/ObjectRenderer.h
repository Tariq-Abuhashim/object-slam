/**
* Tariq updated - Oct, 2022
**/

#ifndef OBJECTRENDERER_H
#define OBJECTRENDERER_H

#include "Renderer.hpp"
#include <pangolin/geometry/geometry.h>
#include <pangolin/geometry/glgeometry.h>

namespace ORB_SLAM3 {

class Object {
public:

    Object(const std::string &path_mesh);

    Object(const Eigen::MatrixXf &vertices, const Eigen::MatrixXi &faces);

    pangolin::GlGeometry object_gl;
    Eigen::Vector3f mean;
    Eigen::Vector3f stddev;
    float norm_factor;
};

// typedef std::shared_ptr<Object> ObjectPointer;

class ObjectRenderer {
public:
    ObjectRenderer(size_t w, size_t h, bool offscreen = false);

    void SetupCamera(double fx, double fy, double cx, double cy, double near, double far);

    uint64_t AddObject(const std::string &mesh_path);

    uint64_t AddObject(const Eigen::MatrixXf &vertices,
                       const Eigen::MatrixXi &faces);

    void Render(uint64_t identifier, const Eigen::Matrix4f &T_co, std::tuple<float, float, float> color);

    void Render(const Object &object, uint64_t identifier, const Eigen::Matrix4f &T_co,
                std::tuple<float, float, float> color);

    inline void DownloadColor(void *ptr_color) {
        renderer->DownloadColor(ptr_color);
    }

    inline int NumObjects() {
        return objects.size();
    }

    inline void Clear() {
        renderer->Clear();
    }

    inline size_t GetWidth() const {
        return renderer->GetWidth();
    }

    inline size_t GetHeight() const {
        return renderer->GetHeight();
    }


private:
    inline uint64_t GetNextIdentifier() {
        if (objects.empty())
            return 0;
        return objects.rbegin()->first + 1;
    }

private:
    double fx, fy, cx, cy, near, far;
    size_t w, h;
    std::map<uint64_t, Object *> objects;
    Renderer *renderer;
};

}

#endif //OBJECTRENDERER_H