
#include "ObjectDrawer.h"

namespace ORB_SLAM3
{

ObjectDrawer::ObjectDrawer(Atlas *pAtlas, MapDrawer *pMapDrawer, const string &strSettingPath) : mpAtlas(pAtlas), mpMapDrawer(pMapDrawer)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    mViewpointF = fSettings["Viewer.ViewpointF"];
    mvObjectColors.push_back(std::tuple<float, float, float>({230. / 255., 0., 0.}));	 // red  0
    mvObjectColors.push_back(std::tuple<float, float, float>({60. / 255., 180. / 255., 75. / 255.}));   // green  1
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 0., 255. / 255.}));	 // blue  2
    mvObjectColors.push_back(std::tuple<float, float, float>({255. / 255., 0, 255. / 255.}));   // Magenta  3
    mvObjectColors.push_back(std::tuple<float, float, float>({255. / 255., 165. / 255., 0}));   // orange 4
    mvObjectColors.push_back(std::tuple<float, float, float>({128. / 255., 0, 128. / 255.}));   //purple 5
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 255. / 255., 255. / 255.}));   //cyan 6
    mvObjectColors.push_back(std::tuple<float, float, float>({210. / 255., 245. / 255., 60. / 255.}));  //lime  7
    mvObjectColors.push_back(std::tuple<float, float, float>({250. / 255., 190. / 255., 190. / 255.})); //pink  8
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 128. / 255., 128. / 255.}));   //Teal  9
    SE3Tcw = Eigen::Matrix4f::Identity();
    SE3TcwFollow = Eigen::Matrix4f::Identity();
}

void ObjectDrawer::SetRenderer(ObjectRenderer *pRenderer)
{
    mpRenderer = pRenderer;
}

void ObjectDrawer::AddObject(MapObject *pMO)
{
    unique_lock<mutex> lock(mMutexObjects);
    mlNewMapObjects.push_back(pMO);
}

void ObjectDrawer::ProcessNewObjects()
{
    unique_lock<mutex> lock(mMutexObjects);
    auto pMO = mlNewMapObjects.front();
    if (pMO)
    {
        int renderId = (int) mpRenderer->AddObject(pMO->vertices, pMO->faces);
        pMO->SetRenderId(renderId);
        mlNewMapObjects.pop_front();
    }
}

void ObjectDrawer::DrawObjects(bool bFollow, const Eigen::Matrix4f &Tec) // Tariq - for ORB_SLAM2
{
    unique_lock<mutex> lock(mMutexObjects);

    auto mvpMapObjects = mpAtlas->GetAllMapObjects();

    for (MapObject *pMO : mvpMapObjects)
    {
        if (!pMO)
            continue;
        if (pMO->isBad())
            continue;

        Eigen::Matrix4f Sim3Two = pMO->GetPoseSim3();
        int idx = pMO->GetRenderId();

        if (bFollow) {
            SE3TcwFollow = SE3Tcw;
        }
        if (pMO->GetRenderId() >= 0)
        {
/*
			cout << "Object: " << pMO->GetRenderId() << endl;
			cout << "     Tec: " << endl;
			cout << "     " << Tec << endl;
			cout << "     SE3TcwFollow: " << endl;
			cout << "     " << SE3TcwFollow << endl;
			cout << "     Sim3Two: " << endl;
			cout << "     " << Sim3Two << endl;
*/
            mpRenderer->Render(idx, Tec * SE3TcwFollow * Sim3Two, mvObjectColors[pMO->GetRenderId() % mvObjectColors.size()]);
        }
        DrawCuboid(pMO);
    }
}

void ObjectDrawer::DrawObjects(bool bFollow) // Tariq - for ORB_SLAM3
{
    unique_lock<mutex> lock(mMutexObjects);

    auto mvpMapObjects = mpAtlas->GetAllMapObjects();

    for (MapObject *pMO : mvpMapObjects)
    {
        if (!pMO)
            continue;
        if (pMO->isBad())
            continue;

        Eigen::Matrix4f Sim3Two = pMO->GetPoseSim3();
		//Eigen::Matrix4f Sim3Two = pMO->SE3Two;
        int idx = pMO->GetRenderId();

        //if (bFollow) {
        //    SE3TcwFollow = SE3Tcw;
        //}
        if (pMO->GetRenderId() >= 0)
        {
            mpRenderer->Render(idx, SE3TcwFollow * Sim3Two, mvObjectColors[pMO->GetRenderId() % mvObjectColors.size()]);
        }
        DrawCuboid(pMO);
    }
}

void ObjectDrawer::DrawCuboid(MapObject *pMO)
{
    const float w=(pMO->w)/2; // openGL takes w/2, h/2, l/2 dimensions
    const float h=(pMO->h)/2;
    const float l=(pMO->l)/2;

    glPushMatrix();

    pangolin::OpenGlMatrix Two=Converter::toMatrixPango(pMO->SE3Two);
#ifdef HAVE_GLES
    glMultMatrixf(Two.m);
#else
    glMultMatrixd(Two.m);
#endif

    const float mCuboidLineWidth=3.0;
    glLineWidth(mCuboidLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);

    glVertex3f(w,h,l);
    glVertex3f(w,-h,l);

    glVertex3f(-w,h,l);
    glVertex3f(-w,-h,l);

    glVertex3f(-w,h,l);
    glVertex3f(w,h,l);

    glVertex3f(-w,-h,l);
    glVertex3f(w,-h,l);

    glVertex3f(w,h,-l);
    glVertex3f(w,-h,-l);

    glVertex3f(-w,h,-l);
    glVertex3f(-w,-h,-l);

    glVertex3f(-w,h,-l);
    glVertex3f(w,h,-l);

    glVertex3f(-w,-h,-l);
    glVertex3f(w,-h,-l);

    glVertex3f(w,h,-l);
    glVertex3f(w,h,l);

    glVertex3f(-w,h,-l);
    glVertex3f(-w,h,l);

    glVertex3f(-w,-h,-l);
    glVertex3f(-w,-h,l);

    glVertex3f(w,-h,-l);
    glVertex3f(w,-h,l);

    glEnd();

    glPopMatrix();
}

void ObjectDrawer::SetCurrentCameraPose(const Eigen::Matrix4f &Tcw)
{
    unique_lock<mutex> lock(mMutexObjects);
    SE3Tcw = Tcw;
}

}

