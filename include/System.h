#ifndef SYSTEM_H
#define SYSTEM_H

// include标准库的头文件
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

// include自定义的头文件
#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"

namespace ORB_SLAM2
{
    class Viewer;
    class FrameDrawer;
    class Map;
    class Tracking;
    class LocalMapping;
    class LoopClosing;

    class System
    {
    // 公共成员变量
    public:
        enum eSensor // 输入传感器的类型
        {
            MONOCULAR = 0;
            STEREO = 1;
            RGBD = 2;
        };
    
    // 公共成员函数
    public:
        // 初始化SLAM系统，启动Local Mapping, Loop Closing 和 Viewer 3个线程。
        System(const string & strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

        // 处理立体相机传入的图像（左、右两个相机）。图像必须同步且经过畸变矫正后才可处理。图像均转换为灰度形式进行处理。
        // 该函数返回相机的位姿，若位姿矩阵为空，则说明tracking失败。
        cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

        // 处理RGBD相机传入的图像。输入depthmap的类型：Float (CV_32F)
        // 该函数返回相机的位姿，若位姿矩阵为空，则说明tracking失败。
        cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

        // 处理单目相机传入的图像。
        // 该函数返回相机的位姿，若位姿矩阵为空，则说明tracking失败。
        cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

        // 停止local mapping线程（建图），只执行tracking。
        void ActivateLocalizationMode();

        // 重新开始(resume)local mapping线程，继续执行SLAM。
        void DeactivateLocalizationMode();

        // 重置系统并清空地图
        void Reset();

        // 等待所有的线程结束，并关闭所有的线程。
        // 必须要在执行此函数后才能执行保存地图数据的各种操作。
        void Shutdown();

        // 以TUM RGB-D的数据格式保存相机轨迹。详见：http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveTrajectoryTUM(const string &filename);

        // 保存关键帧的相机轨迹。单目相机可用此函数。
        void SaveKeyFrameTrajectoryTUM(const string &filename);

        // 以KITTI数据集的格式保存相机轨迹。详见：http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveTrajectoryKITTI(const string &filename);

    // 私有成员变量
    private:
        // 输入传感器的类型
        eSensor mSensor;

        // ORB字典，用于位置识别和特征匹配。
        ORBVocabulary* mpVocabulary;

        // 关键帧数据库，用于位置识别（重定位 和 闭环检测）
        KeyFrameDatabase* mpKeyFrameDatabase;

        // 存放所有KeyFrames和MapPoints的指针。
        Map* mpMap;

        // Tracker. 它接收一帧图像，计算相应的相机位姿。
        // 它也判断何时适合插入一个新的关键帧、建立新的MapPoints、并在tracking失败时执行重定位。
        Tracking* mpTracker;

        // Local Mapper. 它管理局部地图，并执行局部bundle adjustment.
        LocalMapping* mpLocalMapper;
        
        // Loop Closer. 当每插入一个新的关键帧时，它就开始搜索相机的路径是否形成了闭环。(It searches loops with every new keyframe.)
        // 若检测到闭环，则它在一个新的线程中执行 位姿图优化 和 全局bundle adjustment.
        LoopClosing* mpLoopCloser;

        // 用于借助Pangolin绘制地图和当前帧相机的位姿。
        Viewer* mpViewer;

        FrameDrawer* mpFrameDrawer;
        MapDrawer* mpMapDrawer;

        // 系统的3个线程：Local Mapping, Loop Closing, Viewer.
        // Tracking线程在主线程中
        std::thread* mptLocalMapping;
        std::thread* mptLoopClosing;
        std::thread* mptViewer;

        // Reset flag
        std::mutex mMutexReset;
        bool mbReset;

        // 改变系统模式的flag
        std::mutex mMutexMode;
        bool mbActivateLocalizationMode;
        bool mbDeactivateLocalizationMode;
    };
} // namespace ORB_SLAM2

#endif // SYSTEM_H