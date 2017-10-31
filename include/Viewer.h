#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"

// 包含不可重入的互斥锁，主要是为了使对共享资源的互斥使用，即同时只能有一个线程使用，以防止同时使用可能造成的数据问题。
#include <mutex> 
namespace ORB_SLAM2
{
    class Tracking;
    class FrameDrawer;
    class MapDrawer;
    class System;

    class Viewer;
    {
    public:
        Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking* pTracking, const string &strSettingPath);

        // 主线程的函数。功能：描点，记录关键帧，当前相机位姿和上一帧处理的图像。
        void Run();

        void RequestFinish();
        void RequestStop();
        bool isFinished();
        bool isStopped();
        void Release();

    private:
        bool Stop();
        System* mpSystem;
        FrameDrawer* mpFrameDrawer;
        MapDrawer* mpMapDrawer;
        Tracking* mpTracker;

        // 1/fps in ms
        double mT;
        float mImageWidth, mImageHeight;
        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        std::mutex mMutexStop;
    };
} // namespace ORB_SLAM2

# endif // VIEWER_H