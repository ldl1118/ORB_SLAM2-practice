#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h" // 头文件可以相互包含吗？system.h中包含了tracking.h

#include<mutex>

namespace ORB_SLAM2
{
    class Viewer;
    class FrameDrawer;
    class Map;
    class LocalMapping;
    class LoopClosing;
    class System;

    class Tracking
    {
    // 共有成员函数
    public:
        // 变量前缀为p，表示指针类型
        // 变量前缀位str，表示string类型
        Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
                 KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

        // 预处理输入图像并调用Tracking.cc中的Track()函数，提取特征并进行匹配。
        cv::Mat GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp);
        cv::Mat GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp);
        cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

        void SetLocalMapper(LocalMapping* pLocalMapper);
        void SetLoopClosing(LoopClosing* pLoopClosing);
        void SetViewer(Viewer* pViewer);
        
        // 改变MapPoint::PredictScale，考虑焦距对scale的影响。
        void ChangeCalibration(const string &strSettingPath);

        // 当关闭Local Mapping线程，并只希望给相机定位时，使用此函数。
        void InformOnlyTracking(const bool &flag);

    //共有成员变量
    public:
        // Tracking状态
        enum eTrackingState
        {
            SYSTEM_NOT_READY = -1;
            NO_IMAGES_YET = 0;
            NOT_INITIALIZED = 1;
            OK = 2;
            LOST = 3;
        };

        eTrackingState mState;
        eTrackingState mLastProcessedState;

        // 相机的类型：单目，立体，RGBD
        int mSensor;

        // 当前帧
        Frame mCurrentFrame;
        cv::Mat mImGray;

        // 初始化变量（单目）
        // 初始化前两帧的相关变量
        std::vector<int> mvIniLastMatches;
        std::vector<int> mvIniMatches; // 跟踪初始化时前两帧之间的匹配
        std::vector<cv::Point2f> mvbPrevMatched;
        std::vector<cv::Point3f> mvIniP3D;
        Frame mInitialFrame;

        // list用于在函数执行结束后恢复相机的全部路径
        // 一般地，我们对于每一帧图像，保存 与它有关的参考关键帧 和 该帧与参考帧的相对转换矩阵
        list<cv::Mat> mlRelativeFramePoses;
        list<KeyFrame*> mlpReferences;
        list<double> mlFrameTimes;
        list<bool> mlbLost;

        // 当关闭local mpping，只执行localization模式时，该变量为true
        bool mbOnlyTracking;

        void Reset();

    protected:
        // Tracking线程的主要函数
        void Track();

        // 对立体和RGB-D相机输入图像的初始化
        void StereoInitialization();

        // 对单目相机输入图像的初始化
        void MonocularInitialization();
        void CreateInitialMapMonocular();

        void CheckReplacedInLastFrame();
        bool TrackReferenceKeyFrame();
        void UpdateLastFrame();
        bool TrackWithMotionModel();

        bool Relocalization();

        void UpdateLocalMap();
        void UpdateLocalPoints();
        void UpdateLocalKeyFrames();

        bool TrackLocalMap();
        void SearchLocalPoints();

        bool NeedNewKeyFrame();
        void CreateNewKeyFrame();

        // 当仅执行localization模式时，若没有特征点与地图中的点匹配，则该flag为true。
        // 然而，当有足够多的临时匹配点时，tracking仍然可以继续进行。这时我们其实在做V0(Visual Odometry)
        // 系统会尝试通过执行relocalization来得到在地图中localization零漂移的效果
        bool mbVO;

        // 其他的线程指针
        LocalMapping* mpLocalMapper;
        LoopClosing* mpLoopClosing;

        // ORB特征提取器。不管单目还是双目，mpORBextractorLeft都要用到
        // 如果是双目，则要用到mpORBextractorRight
        // 如果是单目，在初始化的时候使用mpIniORBextractor，而不是mpORBextractorLeft。
        // mpIniORBextractor属性中提取的特征点个数是mpORBextractorLeft的两倍
        ORBextractor *mpORBextractorLeft, *mpORBextractorRight;
        ORBextractor* mpIniORBextractor;

        // BoW (Bag-of-Words)词袋
        ORBVocabulary* mpORBVocabulary;
        KeyFrameDatabase* mpKeyFrameDB;
        
        // 单目初始器
        Initializer* mpInitializer;

        // 局部地图
        KeyFrame* mpReferenceKF; // 当前关键帧就是参考帧
        std::vector<KeyFrame*> mvpLocalKeyFrames;
        std::vector<MapPoint*> mvpLocalMapPoints;

        // 系统
        System* mpSystem;

        // 可视化
        Viewer* mpViewer;
        FrameDrawer* mpFrameDrawer;
        MapDrawer* mpMapDrawer;

        // 地图
        Map* mpMap;

        // 用于标定的校正矩阵
        cv::Mat mK;
        cv::Mat mDistCoef;
        float mbf;

        // 插入新关键帧的规则(与fps有关)
        int mMinFrames;
        int mMaxFrames;

        // 远/近点的阈值
        // 我们认为立体/RGB-D相机观察到的较近的点可靠性较高，只通过一帧图像即可插入局部地图
        // 而较远的点需要两个关键帧进行匹配才能决定
        float mThDepth;

        // 只针对RGB-D的输入
        float mDepthMapFactor;

        // 一帧中当前的匹配点数量
        int mnMatchesInliers;

        KeyFrame* mpLastKeyFrame;
        Frame mLastFrame;
        unsigned int mnLastKeyFrameId;
        unsigned int mnLastRelocFrameId;

        // 匀速运动模型
        cv::Mat mVelocity;

        // 颜色等级  RGB：true  BGR：false  若是灰度图像，则忽略此项
        bool mbRGB;

        list<MapPoint*> mlpTemporalPoints;
    };
} // namespace ORB_SLAM2

# endif // TRACKING_H